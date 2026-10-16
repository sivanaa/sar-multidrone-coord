from coordination_node.cbba import UNASSIGNED, CbbaAgent, Task


def _task(task_id, x, y):
    return Task(task_id, (x, y), 'person', 0.9)


def _agent(drone_id, *tasks):
    agent = CbbaAgent(drone_id)
    for task in tasks:
        agent.add_task(task)
    return agent


def _consensus(starts, tasks, rounds=10):
    agents = [_agent(i, *tasks) for i in range(len(starts))]
    for _ in range(rounds):
        for agent, start in zip(agents, starts):
            agent.build_bundle(start)
        states = [(a.drone_id, list(a.tasks),
                   [a.winning_bids[t] for t in a.tasks],
                   [a.winning_agent[t] for t in a.tasks],
                   [a.update_time[t] for t in a.tasks]) for a in agents]
        for agent in agents:
            for state in states:
                if state[0] != agent.drone_id:
                    agent.receive_bundle_state(*state)
    return agents


def test_closer_drone_bids_more():
    target = _task(1, 4.0, 0.0)
    assert (_agent(0, target).bid_for(target, (2.0, 0.0))
            > _agent(1, target).bid_for(target, (0.0, 0.0)))


def test_drone_already_committed_elsewhere_bids_less():
    far, new = _task(1, 0.0, 10.0), _task(2, 3.0, 0.0)
    busy = _agent(0, far)
    busy.build_bundle((0.0, 0.0))
    busy.add_task(new)  # detected after it committed to `far`
    idle = _agent(1, new)
    assert busy.bid_for(new, (0.0, 0.0)) < idle.bid_for(new, (0.0, 0.0))


def test_target_on_the_way_is_flown_first():
    far, on_the_way = _task(1, 10.0, 0.0), _task(2, 4.0, 0.0)
    agent = _agent(0, far, on_the_way)
    agent.build_bundle((0.0, 0.0))
    assert agent.path == [2, 1]


def test_outbid_drone_withdraws_its_claim_on_later_tasks():
    first, second = _task(1, 1.0, 0.0), _task(2, 2.0, 0.0)
    agent = _agent(0, first, second)
    agent.build_bundle((0.0, 0.0))
    assert agent.bundle == [1, 2]
    agent.receive_bundle_state(1, [1], [5.0], [1], [agent.update_time[1] + 1])
    assert agent.bundle == [] and agent.path == []
    assert agent.winning_agent[1] == 1
    # Before 2026-10-06 this stayed "drone 0 wins task 2", so no other
    # drone could ever bid for it.
    assert agent.winning_agent[2] == UNASSIGNED
    assert agent.winning_bids[2] == 0.0


def test_two_drones_split_a_cluster_and_agree():
    tasks = [_task(1, 1.0, 1.0), _task(2, 2.0, 0.0), _task(3, 3.0, 3.0)]
    agents = _consensus([(0.0, 0.0), (4.0, 4.0)], tasks)
    owners = {t: a.drone_id for a in agents for t in a.bundle}
    assert sorted(owners) == [1, 2, 3]            # every target assigned once
    assert set(owners.values()) == {0, 1}         # not all to one drone
    for agent in agents:                          # and both drones agree
        assert all(agent.winning_agent[t] == owners[t] for t in owners)


def test_battery_scales_bids_down_to_nothing_at_reserve():
    from coordination_node.cbba import battery_value_scale
    assert battery_value_scale(0.9) == battery_value_scale(0.5) == 1.0
    assert 0.0 < battery_value_scale(0.4) < 1.0
    assert battery_value_scale(0.25) == battery_value_scale(0.1) == 0.0


def test_low_battery_drone_loses_a_target_it_is_closer_to():
    target = _task(1, 3.0, 0.0)
    agents = _consensus([(2.0, 0.0), (0.0, 0.0)], [target], rounds=1)
    assert agents[0].bundle == [1]                  # healthy: closer wins
    low = _agent(0, target)
    low.value_scale = 0.3
    healthy = _agent(1, target)
    assert low.bid_for(target, (2.0, 0.0)) < healthy.bid_for(target, (0.0, 0.0))


def test_drone_at_reserve_wins_nothing_even_unopposed():
    target = _task(1, 1.0, 0.0)
    agent = _agent(0, target)
    agent.value_scale = 0.0
    assert agent.build_bundle((0.0, 0.0)) == []


def test_release_all_hands_every_task_to_the_other_drone():
    tasks = [_task(1, 1.0, 0.0), _task(2, 2.0, 0.0)]
    agents = _consensus([(0.0, 0.0), (8.0, 0.0)], tasks)
    assert sorted(agents[0].bundle) == [1, 2]
    released = agents[0].release_all()
    assert sorted(released) == [1, 2] and agents[0].bundle == []
    agents[0].value_scale = 0.0
    state = (0, list(agents[0].tasks),
             [agents[0].winning_bids[t] for t in agents[0].tasks],
             [agents[0].winning_agent[t] for t in agents[0].tasks],
             [agents[0].update_time[t] for t in agents[0].tasks])
    agents[1].receive_bundle_state(*state)
    agents[1].build_bundle((8.0, 0.0))
    assert sorted(agents[1].bundle) == [1, 2]


def test_simultaneous_claims_go_to_the_higher_bid_not_the_later_one():
    target = _task(1, 3.0, 0.0)
    near, far = _agent(0, target), _agent(1, target)
    near.build_bundle((2.0, 0.0))
    far.build_bundle((0.0, 0.0))   # claims a moment later, with a lower bid
    assert far.update_time[1] >= near.update_time[1]
    near_state = (0, [1], [near.winning_bids[1]], [0], [near.update_time[1]])
    far_state = (1, [1], [far.winning_bids[1]], [1], [far.update_time[1]])
    near.receive_bundle_state(*far_state)
    far.receive_bundle_state(*near_state)
    assert near.bundle == [1] and far.bundle == []
    assert near.winning_agent[1] == far.winning_agent[1] == 0


def test_a_target_another_drone_finished_is_never_held_or_won_again():
    target = _task(1, 1.0, 0.0)
    holder = _agent(1, target)
    holder.build_bundle((0.0, 0.0))
    assert holder.bundle == [1]
    assert holder.learn_completed([1]) == [1]       # drone 0 got there first
    assert holder.bundle == [] and holder.path == []
    assert holder.build_bundle((0.0, 0.0)) == []
    assert holder.learn_completed([1]) == []        # not news twice


def test_task_types_force_the_crossing_assignment():
    # Each target sits beyond the OTHER drone, so on distance alone each
    # drone takes the one behind itself and nobody crosses. Restricting
    # types makes them swap ends, which is what sim.sh test-x relies on.
    ne = Task(1, (9.0, 9.0), 'x0', 0.9)
    sw = Task(2, (-4.0, -4.0), 'x1', 0.9)
    agents = [CbbaAgent(0, task_types=['x0']), CbbaAgent(1, task_types=['x1'])]
    starts = [(0.0, 0.0), (5.0, 5.0)]
    for agent in agents:
        agent.add_task(ne)
        agent.add_task(sw)
    assert agents[0].bid_for(sw, starts[0]) == 0.0
    for _ in range(3):
        for agent, start in zip(agents, starts):
            agent.build_bundle(start)
        states = [(a.drone_id, [1, 2], [a.winning_bids[t] for t in (1, 2)],
                   [a.winning_agent[t] for t in (1, 2)],
                   [a.update_time[t] for t in (1, 2)]) for a in agents]
        agents[0].receive_bundle_state(*states[1])
        agents[1].receive_bundle_state(*states[0])
    assert agents[0].bundle == [1] and agents[1].bundle == [2]


def test_no_task_types_means_any_type():
    agent = _agent(0, Task(1, (1.0, 0.0), 'anything', 0.9))
    assert agent.build_bundle((0.0, 0.0)) == [1]


def test_duplicate_reports_merge_to_the_lower_id_on_every_drone(monkeypatch):
    import coordination_node.cbba as cbba
    monkeypatch.setattr(cbba, 'DUPLICATE_RADIUS_M', 2.0)
    first, second = _task(7, 3.0, 3.0), _task(100002, 3.5, 3.2)
    a, b = CbbaAgent(0), CbbaAgent(1)
    assert a.add_task(first) and not a.add_task(second)     # heard 7 first
    assert b.add_task(second)
    b.build_bundle((3.0, 3.0))
    assert b.bundle == [100002]
    assert b.add_task(first)                                # lower id wins
    assert 100002 not in b.tasks and b.bundle == []
    assert set(a.tasks) == set(b.tasks) == {7}
    assert not b.add_task(second)                           # stays dropped


def test_a_report_near_a_finished_target_is_dropped(monkeypatch):
    import coordination_node.cbba as cbba
    monkeypatch.setattr(cbba, 'DUPLICATE_RADIUS_M', 2.0)
    agent = _agent(0, _task(1, 1.0, 0.0))
    agent.mark_task_done(1)
    assert not agent.add_task(_task(2, 1.5, 0.0))


def test_outbid_margin_stops_a_slightly_higher_bid_taking_a_task():
    target = _task(1, 4.0, 0.0)
    held = _agent(1, target)
    held.receive_bundle_state(0, [1], [0.70], [0], [1.0])  # drone 0 holds it
    bid = held.bid_for(target, (0.6, 0.0))
    assert 0.70 < bid < 0.77                               # a bit higher
    assert held._outbids(1, bid)                           # plain CBBA: take it
    held.outbid_margin = 0.1
    assert not held._outbids(1, bid)                       # needs > 0.77
    assert held._outbids(1, 0.80)


def test_a_held_bid_rises_as_the_drone_closes_in_and_never_falls():
    target = _task(1, 6.0, 0.0)
    held = _agent(0, target)
    held.build_bundle((0.0, 0.0))
    won_with = held.winning_bids[1]
    assert held.refresh_bids((4.0, 0.0))                   # 2m away now
    assert held.winning_bids[1] > won_with
    # A rival 3m away would have beaten the bid it won with, not this one.
    rival = CbbaAgent(1)
    rival.add_task(target)
    rival.outbid_margin = 0.1
    rival.receive_bundle_state(0, [1], [won_with], [0], [1.0])
    assert rival._outbids(1, rival.bid_for(target, (3.0, 0.0)))
    rival.receive_bundle_state(0, [1], [held.winning_bids[1]], [0], [2.0])
    assert not rival._outbids(1, rival.bid_for(target, (3.0, 0.0)))
    risen = held.winning_bids[1]
    assert not held.refresh_bids((0.0, 0.0))               # flew away: kept
    assert held.winning_bids[1] == risen
