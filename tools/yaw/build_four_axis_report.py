#!/usr/bin/env python3
"""Render the 2026-10-29 CH1/CH4 tethered experiment as offline HTML.

This report describes a particular hardware experiment. It never fits plant
parameters, interpolates missing samples, or writes to the flight controller.
Plot dependency: matplotlib==3.11.2 (used for this delivered report).
"""
import argparse
import base64
import collections
import csv
from datetime import datetime
import hashlib
import html
import io
import json
import math
from pathlib import Path
import re
import statistics
import struct
import subprocess
from zoneinfo import ZoneInfo
import zlib


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def describe(values):
    values = sorted(values)
    return dict(n=len(values), minimum=values[0], maximum=values[-1],
                mean=statistics.mean(values), rms=math.sqrt(statistics.mean(x*x for x in values)),
                p95=values[int(.95*(len(values)-1))])


def angle_error(v):
    return (v[7]-v[6]+180) % 360-180


def load_frames(path):
    data = Path(path).read_bytes()
    if data[:8] != b'RMPLOG1\n':
        raise ValueError('Not an RMPLOG recording')
    offset, counts, states, boots = 8, collections.Counter(), [], set()
    while offset < len(data):
        _, size = struct.unpack_from('<QI', data, offset)
        offset += 12
        frame = data[offset:offset+size]
        offset += size
        if len(frame) != size or frame[:4] != b'RMP1' or \
                zlib.crc32(frame[:-4]) != struct.unpack_from('<I', frame, size-4)[0]:
            raise ValueError('Truncated frame or CRC failure')
        kind = frame[5]
        counts[kind] += 1
        boots.add(struct.unpack_from('<I', frame, 12)[0])
        if kind == 17:
            payload, values = frame[28:-4], {}
            for i in range(payload[4]):
                ident, typ, _, bits = struct.unpack_from('<HBBI', payload, 8+8*i)
                values[ident] = struct.unpack('<f', struct.pack('<I', bits))[0] if typ == 1 else bits
            states.append({'t': struct.unpack_from('<Q', frame, 20)[0]/1e6, 'v': values})
    if offset != len(data) or len(boots) != 1:
        raise ValueError('Invalid boundary or multiple boot sessions')
    return counts, sorted(states, key=lambda s: s['t']), next(iter(boots))


def load_csv(path):
    with open(path, newline='') as stream:
        rows = [dict(t=int(r['target_timestamp_us'])/1e6, seq=int(r['sample_sequence']),
                     boot=int(r['boot_id']), v=[float(r[f'ch{i:02}']) for i in range(1, 33)])
                for r in csv.DictReader(stream)]
    if not rows or not all(math.isfinite(v) for r in rows for v in r['v']):
        raise ValueError('Empty or non-finite telemetry')
    return rows


def is_running(row):
    return all(x > 0 for x in row['v'][23:26])


def excursion_events(rows):
    events = []
    for axis, ch in [('Yaw', 19), ('Pitch', 5), ('Roll', 4)]:
        groups, group = [], []
        for row in rows:
            active = is_running(row) and abs(row['v'][ch]-1500) > 20
            if active:
                if group and (row['t']-group[-1]['t'] > .05 or
                              (row['v'][ch]-1500)*(group[-1]['v'][ch]-1500) < 0):
                    groups.append(group)
                    group = []
                group.append(row)
            elif group:
                groups.append(group)
                group = []
        if group:
            groups.append(group)
        for group in groups:
            if group[-1]['t']-group[0]['t'] < .15:
                continue
            events.append(dict(axis=axis, channel=ch, start=group[0]['t'], end=group[-1]['t'],
                               raw=describe([r['v'][ch] for r in group]), samples=len(group)))
    events.sort(key=lambda e: e['start'])
    for i, event in enumerate(events):
        release = next(r for r in rows if r['t'] > event['end'])
        until = min(release['t']+3, events[i+1]['start'] if i+1<len(events) else math.inf)
        after = [r for r in rows if release['t'] <= r['t'] < until and is_running(r)]
        during = [r for r in rows if event['start'] <= r['t'] <= event['end']]
        event.update(release=release['t'], release_heading=release['v'][6],
                     post_window_s=[release['t'], after[-1]['t']],
                     during_roll=describe([r['v'][15] for r in during]),
                     during_pitch=describe([r['v'][16] for r in during]),
                     during_heading_rate=describe([r['v'][9] for r in during]),
                     during_rate_target=describe([r['v'][8] for r in during]),
                     during_rate_error=describe([r['v'][8]-r['v'][9] for r in during]),
                     post_roll=describe([r['v'][15] for r in after]),
                     post_pitch=describe([r['v'][16] for r in after]),
                     post_heading_change=describe([(r['v'][6]-release['v'][6]+180)%360-180 for r in after]))
    return events


def analyze(args):
    counts, states, boot = load_frames(args.raw)
    arrival = load_csv(args.csv)
    if {r['boot'] for r in arrival} != {boot}:
        raise ValueError('CSV and raw boot sessions differ')
    before, after = [json.loads(Path(p).read_text()) for p in (args.parameters, args.after_parameters)]
    if before['revision'] != after['revision'] or before['values'] != after['values']:
        raise ValueError('Parameters changed during test')
    metadata = json.loads(Path(args.metadata).read_text())['recording']
    if metadata['id'] != '18e30a9b4d1951d8-2d':
        raise ValueError('The user observations and narrative apply only to this dated recording')
    if metadata['target']['build_id'] != 'drehm-yaw-ch14-20261029' or metadata['profile']['revision'] != 5:
        raise ValueError('This report requires the CH1 yaw / CH4 roll build and profile revision 5')
    seqs = [r['seq'] for r in arrival]
    unique, duplicates = {}, []
    for row in arrival:
        key = (row['boot'],row['seq'])
        if key in unique:
            if row['t'] != unique[key]['t'] or row['v'] != unique[key]['v']:
                raise ValueError('Conflicting data for the same boot/sample sequence')
            duplicates.append(row)
        else:
            unique[key]=row
    rows = sorted(unique.values(), key=lambda r: (r['t'], r['seq']))
    running = [r for r in rows if is_running(r)]
    quality = dict(samples=len(arrival), unique_samples=len(rows), frames=sum(counts.values()), frame_kinds=dict(counts),
                   missing_unique=max(seqs)-min(seqs)+1-len(set(seqs)),
                   duplicates=len(seqs)-len(set(seqs)),
                   duplicates_identical=True,
                   duplicate_target_s=[min(r['t'] for r in duplicates),max(r['t'] for r in duplicates)] if duplicates else None,
                   reversed_arrivals=sum(b['t']<a['t'] for a,b in zip(arrival,arrival[1:])),
                   raw_bytes=Path(args.raw).stat().st_size, raw_sha256=sha(args.raw),
                   csv_sha256=sha(args.csv), all_crc_valid=True,
                   recording_drops=metadata['dropped_frames'], recording_error=metadata['error'])
    state_ranges = {str(i): [min(s['v'][i] for s in states), max(s['v'][i] for s in states)]
                    for i in states[0]['v']}
    holds = []
    for title, begin, end in [('初始中杆',370,376),('左转回中',382,386),('右转回中',393,397),
                              ('Pitch后中杆',400,407),('Roll后中杆',411,413)]:
        sample = [r for r in rows if begin<=r['t']<end]
        if not sample or not all(is_running(r) and all(abs(r['v'][i]-1500)<=20 for i in (4,5,19)) for r in sample):
            raise ValueError('A declared centered-stick interval contains pilot input or stopped motors')
        holds.append(dict(label=title,start=begin,end=end,n=len(sample),
                          error=describe([abs(angle_error(r['v'])) for r in sample]),
                          absolute_rate=describe([abs(r['v'][9]) for r in sample])))
    peak = max(states, key=lambda s: s['v'][32])
    first_peak = next(i for i,s in enumerate(states) if s['v'][32]==peak['v'][32])
    motor_stop = next(r for r in rows if r['t']>running[-1]['t'] and not is_running(r))
    result = dict(recording_id=metadata['id'], boot_id=boot, build=metadata['target']['build_id'],
                  profile_revision=5, parameter_revision=before['revision'], parameters_unchanged=True,
                  parameter_values=before['values'], parameter_snapshot_sha256=sha(args.parameters),
                  generated_at=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(timespec='seconds'),
                  source_commit='463dcf5', test_local_start=datetime.fromtimestamp(metadata['started_unix_ms']/1000,ZoneInfo('Asia/Shanghai')).isoformat(),
                  report_revision=2,
                  time_origin='First telemetry sample with all three nonzero motor commands; not a measured liftoff timestamp.',
                  test_local_end=datetime.fromtimestamp(metadata['ended_unix_ms']/1000,ZoneInfo('Asia/Shanghai')).isoformat(),
                  quality=quality, state_ranges=state_ranges,
                  recording_target_s=[rows[0]['t'],rows[-1]['t']],
                  motor_interval_s=[running[0]['t'],running[-1]['t']],
                  motor_stop=dict(t=motor_stop['t'],throttle_us=motor_stop['v'][18]),
                  motor_samples=len(running),
                  powered=dict(throttle=describe([r['v'][18] for r in running]),
                               heading_abs_error=describe([abs(angle_error(r['v'])) for r in running]),
                               heading_rate=describe([r['v'][9] for r in running]),
                               servo_pwm=describe([r['v'][2] for r in running]),
                               mapped_m3_tilt=describe([r['v'][3] for r in running]),
                               motor_dshot=[describe([r['v'][23+i] for r in running]) for i in range(3)],
                               lift_scale=describe([r['v'][27] for r in running])),
                  timing=dict(period=describe([r['v'][28] for r in rows]),
                              max_work_us=max(r['v'][30] for r in rows),
                              max_period_us=max(r['v'][31] for r in rows)),
                  raw_acceleration_peak=dict(lsb=peak['v'][32],g=peak['v'][32]/4096,
                      first_reporting_window_s=[states[first_peak-1]['t'],states[first_peak]['t']],
                      before_reporting_window_lsb=states[first_peak-1]['v'][32]),
                  events=excursion_events(rows), holds=holds,
                  user_observation='About 46 seconds airborne; 250g counterweight continuously taut; actual order throttle, yaw left/right, pitch, roll; after release fore/aft/lateral swaying obscured perceived reverse commands.',
                  parameter_fit_performed=False,
                  limitations=['No synchronized video or exact liftoff/landing timestamps.',
                               'Unknown tether tension, angle, attachment offset, rope length and torsion.',
                               'No position/height/servo-position/motor-thrust/independent-heading sensors.',
                               'Sorted copies are used only for plotting and interval statistics; original order and gaps remain preserved.'])
    if [e['axis'] for e in result['events']] != ['Yaw','Yaw','Pitch','Roll','Roll']:
        raise ValueError('The dated narrative requires this experiment\'s five verified input excursions')
    return result, rows, states


def table(headers, body):
    esc = lambda s: html.escape(str(s))
    return '<div class="table-wrap"><table><thead><tr>'+''.join('<th>'+esc(x)+'</th>' for x in headers)+\
           '</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+esc(x)+'</td>' for x in row)+'</tr>' for row in body)+'</tbody></table></div>'


def charts(rows, states):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    import numpy as np
    # Embed glyph outlines so plots remain readable on an offline machine.
    font = Path('/System/Library/Fonts/Supplemental/Arial Unicode.ttf')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
    cjk = font_manager.FontProperties(fname=str(font)).get_name() if font.exists() else 'Noto Sans CJK SC'
    font_manager.findfont(font_manager.FontProperties(family=cjk),fallback_to_default=False)
    plt.rcParams.update({'font.family':[cjk,'DejaVu Sans'],'font.size':10,'axes.spines.top':False,
                         'axes.spines.right':False,'axes.grid':True,'grid.alpha':.2,
                         'svg.fonttype':'path','svg.hashsalt':'drehm-four-axis-20261029',
                         'path.simplify':False})
    colors = ['#1d4ed8','#b45309','#047857','#be185d','#6d28d9']
    origin = next(r['t'] for r in rows if is_running(r))
    events = excursion_events(rows)

    def plot(panels, begin, end, highlight=()):
        selected = [r for r in rows if begin<=r['t']<=end]
        xx, yy = [], []
        previous = None
        for r in selected:
            if previous and (r['seq']-previous['seq'] != 1 or r['t']-previous['t']>.015):
                xx.append(math.nan); yy.append([math.nan]*32)
            xx.append(r['t']-origin); yy.append(r['v']); previous=r
        xx, yy = np.array(xx), np.array(yy)
        fig, axes = plt.subplots(len(panels),1,figsize=(11,2.15*len(panels)),sharex=True,squeeze=False,layout='constrained')
        for axis, (title, lines, unit) in zip(axes[:,0],panels):
            for j,(channel,label) in enumerate(lines):
                values = yy[:,channel] if channel>=0 else (yy[:,7]-yy[:,6]+180)%360-180
                axis.plot(xx,values,label=label,color=colors[j],lw=1.05)
            axis.set(title=title,ylabel=unit,xlim=(begin-origin,end-origin))
            for event in events:
                if event['axis'] in highlight:
                    axis.axvspan(event['start']-origin,event['end']-origin,color='#c8d9ee',alpha=.35,zorder=0)
            axis.legend(loc='upper right',ncol=min(4,len(lines)),fontsize=8,framealpha=.92)
        axes[-1,0].set_xlabel('相对时间 t (s)：0 为三电机首次非零，非精确离地时刻')
        output=io.StringIO(); fig.savefig(output,format='svg',metadata={'Date':None}); plt.close(fig)
        return 'data:image/svg+xml;base64,'+base64.b64encode(output.getvalue().encode()).decode()

    return {
      'overview':plot([
          ('操纵顺序：先油门，再偏航、俯仰、横滚',[(18,'油门 CH3'),(19,'Yaw CH1'),(5,'Pitch CH2'),(4,'Roll CH4')],'µs'),
          ('姿态响应',[(15,'Roll 右倾为正'),(16,'Pitch 前倾为正')],'°'),
          ('相对航向',[(6,'IMU 航向'),(7,'航向目标')],'°'),
          ('航向角速度',[(8,'角速度目标'),(9,'实测角速度')],'°/s')],366,416),
      'yaw':plot([
          ('Yaw 输入：数值减小为左转，阴影为离开死区',[(19,'原始 CH1')],'µs'),
          ('航向目标与反馈',[(6,'IMU 航向'),(7,'航向目标')],'°'),
          ('航向差：目标减反馈，按最短角度计算',[(-1,'航向差')],'°'),
          ('内环跟踪：20°/s 为目标限幅',[(8,'角速度目标'),(9,'实测角速度')],'°/s')],369,397,('Yaw',)),
      'pitch':plot([
          ('Pitch 输入：阴影为持续前倾指令',[(5,'Pitch CH2')],'µs'),
          ('俯仰姿态：回中后跨过水平位置',[(16,'Pitch 前倾为正')],'°'),
          ('角速度观测',[(21,'GyroY')],'°/s'),
          ('同时观察 Yaw 耦合',[(6,'IMU 航向'),(7,'航向目标')],'°')],396,407,('Pitch',)),
      'roll':plot([
          ('Roll 输入：右倾后有独立左倾指令',[(4,'Roll CH4')],'µs'),
          ('横滚姿态',[(15,'Roll 右倾为正')],'°'),
          ('角速度观测',[(20,'GyroX')],'°/s'),
          ('同时观察 Yaw 耦合',[(6,'IMU 航向'),(7,'航向目标')],'°')],406,414,('Roll',)),
      'actuators':plot([
          ('舵机 PWM 命令：并非实测位置',[(2,'PWM')],'µs'),
          ('Yaw 力矩：需求与模型预测，不是实测扭矩',[(11,'需求'),(12,'模型预测'),(13,'差值')],'N·m'),
          ('实际发送的电机命令',[(23,'M2'),(24,'M4'),(25,'M3')],'DShot'),
          ('机体 Z 轴低通加速度：含重力响应',[(22,'AccZ，静止约 1g')],'g')],366,416),
      'timing':plot([
          ('主循环周期与累计最大值',[(28,'周期'),(31,'累计最大周期')],'µs'),
          ('工作耗时：不含等待时间',[(29,'上一周期工作'),(30,'累计最大工作')],'µs')],rows[0]['t'],rows[-1]['t'])}


def render(result, figures, template):
    r=result; q=r['quality']; p={int(v['id']):v['value'] for v in r['parameter_values']}
    origin=r['motor_interval_s'][0]
    event_rows=[]
    directions=['左转','右转','前倾','右倾','左倾']
    for label,event in zip(directions,r['events']):
        axis=event['axis']; response=event['during_heading_rate'] if axis=='Yaw' else event['during_pitch'] if axis=='Pitch' else event['during_roll']
        event_rows.append([axis+' '+label,f"{event['start']-origin:.2f}–{event['end']-origin:.2f}",
                           f"{event['end']-event['start']:.2f}",
                           f"{event['raw']['minimum']:.0f}–{event['raw']['maximum']:.0f}",
                           f"{response['minimum']:.2f}–{response['maximum']:.2f}"+(' °/s' if axis=='Yaw' else ' °')])
    hold_rows=[[h['label'],f"{h['start']-origin:.2f}–{h['end']-origin:.2f}",h['n'],f"{h['error']['mean']:.2f}",
                f"{h['error']['p95']:.2f}",f"{h['error']['maximum']:.2f}",f"{h['absolute_rate']['maximum']:.2f}"] for h in r['holds']]
    data={
      'GENERATED':html.escape(r['generated_at']),
      'EVENT_TABLE':table(['机动','相对时间 t / s','持续 / s','原始输入 / µs','机动期间响应范围'],event_rows),
      'HOLD_TABLE':table(['中杆片段','相对时间 t / s','样本','平均绝对差 / °','P95 / °','最大绝对差 / °','最大|航向率| / °/s'],hold_rows),
      'YAW_RATE_TABLE':table(['Yaw机动','目标率范围 / °/s','实测率范围 / °/s','率跟踪差 RMS / °/s'],[
          [label,f"{event['during_rate_target']['minimum']:.2f}–{event['during_rate_target']['maximum']:.2f}",
           f"{event['during_heading_rate']['minimum']:.2f}–{event['during_heading_rate']['maximum']:.2f}",
           f"{event['during_rate_error']['rms']:.2f}"] for label,event in zip(directions[:2],r['events'][:2])]),
      'MODEL_TABLE':table(['参数','本机运行值','证据／限制'],[
          ['质量 / 臂长',f"{p[15]:.3f} kg / 0.500 m",'用户提供，水平重心居中'],
          ['Jz（固件Izz）',f"{p[16]:.4f} kg·m²",'mL²/3粗估初值，非实测'],
          ['κ₂ / κ₄ / κ₃','0.010 / 0.010 / 0.010 m','反扭矩/推力比初值，非辨识结果'],
          ['每电机推力表','0 / 0.4598 / 1.8394 / 4.1386 / 7.3575 N','u=0/.25/.5/.75/1；假设总推重比3'],
          ['舵机 / 电机滞后',f"{p[12]:.3f} / {p[76]:.3f} s",'一阶估算，未测真实位置或转速'],
          ['模型标志','初始模型许可=1；已验证=0','完整分配器运行，不代表物理参数已标定']]),
      'GAINS_TABLE':table(['控制量','运行值'],[
          ['Yaw 航向Kp / 率Kp / 率Ki',f"{p[3]:.2f} s⁻¹ / {p[4]:.2f} s⁻¹ / {p[5]:.2f} s⁻²"],
          ['Yaw指令率 / 目标加速度',f"{p[6]:.0f} °/s / {p[7]:.0f} °/s²"],
          ['航向率低通 / 目标超前角限幅',f"{p[8]:.0f} Hz / ±{p[9]:.0f}°"],
          ['物理倾角 / 舵机逻辑限速',f"±{p[13]:.0f}° / {p[11]:.0f} logical°/s"],
          ['Roll/Pitch有效P / I / D','0.002 / 0.003 / 0.0005；沿用原姿态控制'],
          ['Roll/Pitch倾角上限','30°；驾驶向量径向限幅']]),
      'QUALITY_TABLE':table(['项目','实测'],[
          ['本机录制时间',r['test_local_start']+' 至 '+r['test_local_end']],
          ['目标时钟范围',f"{r['recording_target_s'][0]:.6f}–{r['recording_target_s'][1]:.6f} s"],
          ['遥测样本 / 原始帧',f"{q['samples']:,} / {q['frames']:,}"],
          ['独立遥测样本',f"{q['unique_samples']:,}；完全相同的重复样本只统计一次"],
          ['缺样 / 重复 / 相邻到达倒序',f"{q['missing_unique']} / {q['duplicates']} / {q['reversed_arrivals']}"],
          ['范围内缺样比例',f"{100*q['missing_unique']/(q['unique_samples']+q['missing_unique']):.3f}%"],
          ['原始CRC / Agent录制丢帧','全部有效 / 0'],
          ['参数修订 / 试前试后','2 / 71项完全一致'],
          ['Build / Boot',r['build']+' / '+str(r['boot_id'])],
          ['应用 / 固件提交','drehmflight-yaw-flight revision5 / 463dcf5']]),
      'RAW_SHA':q['raw_sha256'], 'CSV_SHA':q['csv_sha256'],
      'PARAM_SHA':r['parameter_snapshot_sha256'],
      'LEFT_RELEASE_MIN':f"{r['events'][0]['post_heading_change']['minimum']:.2f}",
      'LEFT_RELEASE_MAX':f"{r['events'][0]['post_heading_change']['maximum']:.2f}",
      'RIGHT_RELEASE_MIN':f"{r['events'][1]['post_heading_change']['minimum']:.2f}",
      'RIGHT_RELEASE_MAX':f"{r['events'][1]['post_heading_change']['maximum']:.2f}",
      'METRICS_JSON':html.escape(json.dumps(r,ensure_ascii=False,indent=2)),
    }
    for key,value in figures.items(): data['FIG_'+key.upper()]=value
    output=Path(template).read_text()
    for key,value in data.items(): output=output.replace('{{'+key+'}}',str(value))
    if re.search(r'\{\{[A-Z_]+\}\}',output): raise ValueError('Unfilled template field')
    return output


def render_math(output, root):
    if not (Path(root)/'js/mathjax.js').is_file():
        raise ValueError('Install mathjax-full@3.2.2 with npm install --prefix tools/yaw '
                         'or pass its directory via --mathjax-root')
    pattern=re.compile(r'<div class="math-source">(.*?)</div>',re.S)
    expressions=[html.unescape(tex) for tex in pattern.findall(output)]
    completed=subprocess.run(['node',str(Path(__file__).with_name('render_report_math.cjs')),str(root)],
        input=json.dumps(expressions),text=True,capture_output=True)
    if completed.returncode:
        raise ValueError('TeX-to-SVG rendering failed: '+completed.stderr)
    rendered=json.loads(completed.stdout)
    if len(expressions)!=len(rendered) or not rendered:
        raise ValueError('Formula rendering did not produce one SVG per expression')
    iterator=iter(rendered)
    return pattern.sub(lambda _: '<div class="math-body">'+next(iterator)+'</div>',output)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('csv','raw','parameters','after-parameters','metadata','output'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--mathjax-root',type=Path,
                        default=Path(__file__).with_name('node_modules')/'mathjax-full',
                        help='Build-time mathjax-full@3.2.2 directory (report stays offline)')
    args=parser.parse_args()
    result,rows,states=analyze(args)
    template=Path(__file__).with_name('four_axis_report_template.html')
    output=Path(args.output); output.parent.mkdir(parents=True,exist_ok=True)
    figures=charts(rows,states)
    output.write_text(render_math(render(result,figures,template),args.mathjax_root))
    output.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),bytes=output.stat().st_size,samples=result['quality']['samples'],
                          sha256=sha(output)),ensure_ascii=False))


if __name__=='__main__': main()
