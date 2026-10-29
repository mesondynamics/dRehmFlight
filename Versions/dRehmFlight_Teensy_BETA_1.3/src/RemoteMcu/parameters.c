/* SPDX-License-Identifier: Apache-2.0 */
#include "parameters.h"
#include <math.h>
#include <string.h>

static bool valid_value(const rmp_param_def_t *d, uint32_t bits) {
    double value = bits;
    if (d->type == RMP_PARAM_F32) { float f; memcpy(&f, &bits, 4); value = f; }
    if (d->type == RMP_PARAM_BOOL && bits > 1) return false;
    return isfinite(value) && value >= d->minimum && value <= d->maximum;
}
bool rmp_params_valid(const rmp_params_t *p, const uint32_t *values) {
    for (unsigned i=0; i<p->count; ++i) if (!valid_value(&p->definitions[i], values[i])) return false;
    return true;
}
bool rmp_params_init(rmp_params_t *p, const rmp_param_def_t *defs, unsigned count,
                     rmp_param_save_fn save, void *context) {
    if (!p || !defs || !count || count>RMP_PARAM_MAX) return false;
    memset(p, 0, sizeof(*p)); p->definitions=defs; p->count=count; p->save=save; p->context=context;
    for (unsigned i=0; i<count; ++i) {
        if ((defs[i].name && strlen(defs[i].name)>48) || !isfinite(defs[i].minimum) || !isfinite(defs[i].maximum) || defs[i].minimum>defs[i].maximum || defs[i].type<1 || defs[i].type>3 || !valid_value(&defs[i], defs[i].initial)) return false;
        for (unsigned j=0; j<i; ++j) if (defs[j].id==defs[i].id) return false;
        p->values[i]=defs[i].initial;
    }
    p->revision=1;
    return true;
}
static uint8_t execute(rmp_params_t *p, uint8_t kind, const uint8_t *request, size_t length, bool safe) {
    if (length<16 || request[13] || request[14] || request[15]) return RMP_MALFORMED;
    unsigned count=request[12];
    if (count>p->count || count>RMP_PARAM_BATCH || length!=16+count*8u) return RMP_MALFORMED;
    if (kind==RMP_PARAM_GET) return count || p->count>RMP_PARAM_BATCH ? RMP_MALFORMED : RMP_APPLIED;
    if (kind!=RMP_PARAM_SET && kind!=RMP_PARAM_SAVE) return RMP_MALFORMED;
    if (!safe) return RMP_UNSAFE;
    if (rmp_get_u32(request+8)!=p->revision) return RMP_REVISION_CONFLICT;
    if (kind==RMP_PARAM_SAVE) {
        if (count) return RMP_MALFORMED;
        if (p->saved_revision!=p->revision) {
            if (!p->save || !p->save(p->values,p->count,p->revision,p->context)) return RMP_INTERNAL;
            p->saved_revision=p->revision;
        }
        return RMP_SAVED;
    }
    if (!count) return RMP_MALFORMED;
    uint32_t next[RMP_PARAM_MAX]; memcpy(next,p->values,sizeof(next));
    bool seen[RMP_PARAM_MAX]={false};
    for (unsigned i=0; i<count; ++i) {
        const uint8_t *entry=request+16+i*8;
        unsigned index=0;
        while (index<p->count && p->definitions[index].id!=rmp_get_u16(entry)) ++index;
        if (index==p->count) return RMP_INVALID_ID;
        if (entry[2]!=p->definitions[index].type) return RMP_TYPE_MISMATCH;
        if (entry[3] || seen[index]) return RMP_MALFORMED;
        seen[index]=true;
        next[index]=rmp_get_u32(entry+4);
        if (!valid_value(&p->definitions[index],next[index])) return RMP_OUT_OF_RANGE;
    }
    if (memcmp(next,p->values,p->count*4)) { memcpy(p->values,next,p->count*4); ++p->revision; }
    return RMP_APPLIED;
}
size_t rmp_params_handle(rmp_params_t *p, uint8_t kind, const uint8_t *request,
                         size_t length, bool safe, uint8_t *response) {
    if (!p || !request || !response || length<8 || length>RMP_PARAM_BYTES) return 0;
    uint64_t id=rmp_get_u64(request); bool conflict=false;
    for (unsigned i=0; i<8; ++i) {
        const rmp_param_cache_t *c=&p->cache[i];
        if (!c->valid || c->id!=id) continue;
        if (c->kind==kind && c->request_length==length && !memcmp(c->request,request,length)) {
            memcpy(response,c->response,c->response_length); return c->response_length;
        }
        conflict=true; break;
    }
    bool paged=kind==RMP_PARAM_READ_PAGE || kind==RMP_PARAM_SCHEMA_PAGE;
    uint8_t status=RMP_MALFORMED; unsigned first=0,count=0;
    if (conflict) status=RMP_REQUEST_CONFLICT;
    else if (paged && length==16 && !request[15]) {
        first=rmp_get_u16(request+12); unsigned limit=request[14];
        unsigned maximum=kind==RMP_PARAM_READ_PAGE ? RMP_PARAM_BATCH : 8;
        if (first<p->count && limit && limit<=maximum &&
            (kind!=RMP_PARAM_SCHEMA_PAGE || !rmp_get_u32(request+8))) {
            status=RMP_APPLIED;
            if (kind==RMP_PARAM_READ_PAGE && rmp_get_u32(request+8) &&
                rmp_get_u32(request+8)!=p->revision) status=RMP_REVISION_CONFLICT;
            if (status==RMP_APPLIED) { count=p->count-first; if (count>limit) count=limit; }
        }
    } else if (!paged) status=execute(p,kind,request,length,safe);
    if (conflict) { status=RMP_REQUEST_CONFLICT; count=0; }
    memset(response,0,RMP_PARAM_BYTES); rmp_put_u64(response,id);
    response[8]=status; response[9]=kind;
    rmp_put_u32(response+12,p->revision); rmp_put_u32(response+16,p->saved_revision);
    size_t size=paged ? 24 : 20;
    if (paged) { rmp_put_u16(response+20,(uint16_t)p->count); rmp_put_u16(response+22,(uint16_t)first); }
    else count=p->count<=RMP_PARAM_BATCH ? p->count : 0;
    response[10]=(uint8_t)count;
    for (unsigned i=0; i<count; ++i) {
        const rmp_param_def_t *d=&p->definitions[first+i];
        uint8_t *entry=response+size; rmp_put_u16(entry,d->id); entry[2]=d->type;
        if (kind==RMP_PARAM_SCHEMA_PAGE) {
            size_t n=d->name ? strlen(d->name) : 0; if (n>48) n=48;
            entry[3]=(uint8_t)n; uint64_t bits;
            memcpy(&bits,&d->minimum,8); rmp_put_u64(entry+4,bits);
            memcpy(&bits,&d->maximum,8); rmp_put_u64(entry+12,bits);
            if (n) memcpy(entry+20,d->name,n);
            size+=20+n;
        } else { rmp_put_u32(entry+4,p->values[first+i]); size+=8; }
    }
    if (!conflict) {
        rmp_param_cache_t *c=&p->cache[p->next_cache++ % 8];
        c->valid=true; c->id=id; c->kind=kind; c->request_length=(uint16_t)length;
        c->response_length=(uint16_t)size;
        memcpy(c->request,request,length); memcpy(c->response,response,size);
    }
    return size;
}
