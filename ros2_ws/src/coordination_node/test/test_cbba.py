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
