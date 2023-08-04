
I have N nodes 0,...,N and C checkpoints. They are labeled as follows:

0 (depot), 1, ..., k, ..., k + r **(N)**, ..., k + r + c **(N + C)**.

- The solver can leave the depot and visit nodes [1, k + r] in any way, subject to constraints, say fuel. 
- If (and only if) the fuel runs out **OR** it visits nodes [k + 1, k + r], then it must visit nodes [k + r + 1, k + r + c], known as checkpoint nodes. They are the unload nodes from the [VRP Reload example](https://github.com/google/or-tools/blob/master/ortools/constraint_solver/samples/cvrp_reload.py).
- Then, it restarts from any node [1, k].

That being said, the adjacency rules are:

- From the depot, I can go to nodes [1, k].
- From nodes [1, k], I can go to nodes [1, k + r]
- From nodes [k, k + r], I must go to nodes (checkpoints) [k + r + 1, k + r + c]

The time_matrix figure reports the time travel between the nodes. The values are reported as follows:

1. A list of the node labels.
2. A list of node balance
3. The time matrix itself.
