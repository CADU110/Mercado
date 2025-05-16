# Mercado

## Modelagem

#### Variáveis

```math
\begin{aligned}
& c_n \in \{0, 1\}, && \forall n \in C && \text{(Corredores)} \\
& p_k \in \{0, 1\}, && \forall k \in P && \text{(Pedidos)}
\end{aligned}
```

#### Restrições

```math
LB \leq \left( \sum_{k \in P} u_k \cdot p_k \right) \leq UB
```

```math
\forall i \in I,\quad \sum_{k \in P} u_{ik} \cdot p_k \leq \sum_{j \in C} u_{ij} \cdot c_j
```

#### Objetivo (temporário)

```math
\min \left( \left( \sum_{j \in C} M \cdot c_j \right) - \left( \sum_{k \in P} K \cdot u_k \cdot p_k \right) \right)
```

## Executando

#### Para rodar um dataset inteiro:

Adicionar o caminho do OR-TOOLS ao arquivo `run_challenge.py`;

Então execute:

```
python3 run_challenge.py . ./datasets/<a-b-f> ../output/<a-b-f>
```

#### Para rodar uma instância individual:

Execute:

```
mvn clean package
```

```
timeout 605s java -Djava.library.path=<ortools-library-path> -jar target/ChallengeSBPO2025-1.0.jar <input-file> <output-file>
```