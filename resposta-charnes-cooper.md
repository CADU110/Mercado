The problem described is a fractional programming problem where the objective is to maximize the ratio of the total number of items collected to the number of corridors visited, subject to certain constraints. The Charnes-Cooper transformation is a technique used to convert such fractional programming problems into equivalent linear programming problems, which are easier to solve.

### Problem Recap:
The objective is to maximize:

```math
\max \frac{\sum_{o \in O'} \sum_{i \in I_o} u_{oi}}{|A'|}
```

subject to:
```math
\begin{align}
&\sum_{o \in O'} \sum_{i \in I_o} u_{oi} \geq \text{LB} \\
&\sum_{o \in O'} \sum_{i \in I_o} u_{oi} \leq \text{UB} \\
&\sum_{o \in O'} u_{oi} \leq \sum_{a \in A'} u_{ai}, \quad \forall i \in I_o,\, o \in O' \tag{3}
\end{align}
```

### Charnes-Cooper Transformation Steps:
The Charnes-Cooper transformation involves introducing a new variable \( t > 0 \) such that:

```math
t = \frac{1}{|A'|}
```

Then, we define new variables for the decision variables in the original problem scaled by \( t \). However, since \( |A'| \) is a discrete quantity (the number of corridors), we need to handle this carefully. The transformation is typically applied to continuous variables, but we can adapt it for this case.

#### Step 1: Rewrite the Objective
The objective can be rewritten as:

```math
\max \frac{\sum_{o \in O'} \sum_{i \in I_o} u_{oi}}{|A'|} = \max \sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot t
```

where \( t = \frac{1}{|A'|} \).

#### Step 2: Introduce Auxiliary Variables
Define new variables:

```math
y_o = t \cdot x_o, \quad \forall o \in O'
```

where \( x_o \) is a binary variable indicating whether order \( o \) is included in the wave (\( x_o = 1 \)) or not (\( x_o = 0 \)). Similarly, define:

```math
z_a = t \cdot w_a, \quad \forall a \in A'
```

where \( w_a \) is a binary variable indicating whether corridor \( a \) is visited (\( w_a = 1 \)) or not (\( w_a = 0 \)).

#### Step 3: Rewrite the Constraints
1. The total units constraint becomes:
   
   ```math
   \sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot x_o \geq \text{LB} \implies \sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot y_o \geq \text{LB} \cdot t
   ```
   
   Similarly for the upper bound:
   
   ```math
   \sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot y_o \leq \text{UB} \cdot t
   ```
   

2. The corridor capacity constraint becomes:
   
   ```math
   \sum_{o \in O'} u_{oi} \cdot x_o \leq \sum_{a \in A'} u_{ai} \cdot w_a \implies \sum_{o \in O'} u_{oi} \cdot y_o \leq \sum_{a \in A'} u_{ai} \cdot z_a, \quad \forall i
   ```
   

3. The corridor selection constraint:
   
   ```math
   \sum_{a \in A'} w_a = |A'| \implies \sum_{a \in A'} z_a = t \cdot |A'|
   ```
   
   But since \( t = \frac{1}{|A'|} \), this simplifies to:
   
   ```math
   \sum_{a \in A'} z_a = 1
   ```
   

#### Step 4: Additional Constraints
- The relationship between \( y_o \) and \( t \):
  
  ```math
  y_o \leq t, \quad \forall o \in O'
  ```
  
  This ensures that \( y_o = t \) when \( x_o = 1 \), and \( y_o = 0 \) otherwise.

- The relationship between \( z_a \) and \( t \):
  
  ```math
  z_a \leq t, \quad \forall a \in A'
  ```
  
  This ensures that \( z_a = t \) when \( w_a = 1 \), and \( z_a = 0 \) otherwise.

#### Step 5: Final Transformed Problem
The transformed linear programming problem is:

```math
\max \sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot y_o
```

subject to:
```math
\begin{align*}
&\sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot y_o \geq \text{LB} \cdot t \\
&\sum_{o \in O'} \sum_{i \in I_o} u_{oi} \cdot y_o \leq \text{UB} \cdot t \\
&\sum_{o \in O'} u_{oi} \cdot y_o \leq \sum_{a \in A'} u_{ai} \cdot z_a, \quad \forall i \\
&\sum_{a \in A'} z_a = 1 \\
&y_o \leq t, \quad \forall o \in O' \\
&z_a \leq t, \quad \forall a \in A' \\
&t > 0,\quad y_o \geq 0,\quad z_a \geq 0
\end{align*}
```

### Notes:
- The transformed problem is now a linear program (LP) but still involves binary decisions implicitly through \( y_o \) and \( z_a \). In practice, you may need to use mixed-integer linear programming (MILP) techniques to handle the binary nature of \( x_o \) and \( w_a \).
- The Charnes-Cooper transformation simplifies the fractional objective but does not eliminate the combinatorial nature of the problem entirely. You may still need heuristic or exact methods to solve the resulting LP or MILP efficiently for large instances.

This transformation allows you to leverage linear programming solvers to address the fractional objective in the original problem.
