"""Exclusive offline handoff graph; no appearance-only union of identities."""


def identity_evidence_sufficient(first,second,policy=None):
    policy=policy or {}
    past=first.get('identity_evidence',{});future=second.get('identity_evidence',{})
    # A long, consistently tracked edge silhouette may be resolved by the
    # future full view. A brief cropped glimpse may not identify another car.
    minimum=policy.get('minimum_clear_observations',3)
    past_supported=past.get('usable_observations',minimum)>=minimum or (past.get('observed_samples',0)>=policy.get('minimum_partial_past_observations',40) and past.get('observed_span_s',0)>=policy.get('minimum_partial_past_span_s',8))
    past_supported=past_supported or (past.get('wide_side_body_observations',0)>=8 and past.get('wide_side_body_span_s',0)>=1.4 and past.get('observed_span_s',0)>=3)
    return past_supported and future.get('usable_observations',minimum)>=minimum


def select_handoffs(edges,actors,margin=.15):
    by_id={a['id']:a for a in actors};outgoing={};incoming={}
    for e in edges:
        outgoing.setdefault(e['from'],[]).append(e);incoming.setdefault(e['to'],[]).append(e)
    root={key:key for key in by_id};members={key:{key} for key in by_id}
    def find(k):
        while root[k]!=k:k=root[k]
        return k
    for e in sorted(edges,key=lambda e:e['cost']):
        a,b=e['from'],e['to'];reason=None
        if by_id[b]['start_s']<=by_id[a]['start_s']:
            e['selected']=False;e['reason']='non_chronological_handoff';continue
        if not e.get('identity_eligible',True):
            e['selected']=False;e['reason']='needs_clear_identity_observations';continue
        for options in (outgoing[a],incoming[b]):
            ordered=sorted(options,key=lambda x:x['cost'])
            if ordered[0] is not e:reason='not_mutual_best';break
            if len(ordered)>1 and ordered[1]['cost']-ordered[0]['cost']<margin:reason='ambiguous_alternatives';break
        ra,rb=find(a),find(b)
        if reason is None and ra==rb:reason='redundant_identity_edge'
        if reason is None:
            for x in members[ra]:
                for y in members[rb]:
                    u,v=by_id[x],by_id[y]
                    if min(u['end_s'],v['end_s'])-max(u['start_s'],v['start_s'])>.4:
                        reason='component_simultaneous_observation_conflict';break
                if reason:break
        e['selected']=reason is None;e['reason']=reason or 'exclusive_mutual_best_handoff'
        if e['selected']:
            root[rb]=ra;members[ra]|=members[rb]
    return edges
