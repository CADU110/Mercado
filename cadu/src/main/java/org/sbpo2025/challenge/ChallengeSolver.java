package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.util.*;
import java.util.concurrent.TimeUnit;

import com.google.ortools.Loader;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected Map<Integer, Integer> itens;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;

    private int nOrders;
    private int nAisles;
    private long[] nItemsPerOrder;

    static {
        Loader.loadNativeLibraries();
    }

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    protected void getItemsPerOrder(List<Map<Integer, Integer>> orders) {
        this.nOrders = orders.size();
        this.nItemsPerOrder = new long[this.nOrders];

        for (int i = 0; i < this.nOrders; i++) {
            Map<Integer, Integer> order = orders.get(i);
            long totalItems = 0;

            for (int quantity : order.values()) {
                totalItems += quantity;
            }

            this.nItemsPerOrder[i] = totalItems;
        }
    }

    protected Map<Integer, Integer> getItemsFromOrders(List<Map<Integer, Integer>> orders) {
        Map<Integer, Integer> totalItensMap = new HashMap<>();

        for (Map<Integer, Integer> order : orders) {
            for (Map.Entry<Integer, Integer> entry : order.entrySet()) {
                int itemId = entry.getKey();
                int quantity = entry.getValue();
                totalItensMap.put(itemId, totalItensMap.getOrDefault(itemId, 0) + quantity);
            }
        }

        return totalItensMap;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        MPSolver solver = MPSolver.createSolver("SCIP");

        if (solver == null) {
            System.out.println("Could not create solver SCIP");
            return new ChallengeSolution(Set.of(), Set.of());
        }

        this.nOrders = orders.size();
        this.nAisles = aisles.size();
        this.itens = getItemsFromOrders(orders);

        getItemsPerOrder(orders);

        MPVariable[] p = new MPVariable[nItems];
        MPVariable[] c = new MPVariable[nAisles];

        for (int i = 0; i < nItems; i++) {
            p[i] = solver.makeBoolVar("p_" + i);
        }

        for (int i = 0; i < nAisles; i++) {
            c[i] = solver.makeBoolVar("c_" + i);
        }

        MPConstraint itemsBound = solver.makeConstraint(waveSizeLB, waveSizeUB, "waveSize");
        for (int i = 0; i < nItems; i++) {
            itemsBound.setCoefficient(p[i], itens.getOrDefault(i, 0));
        }

        for (int itemId = 0; itemId < nItems; itemId++) {
            MPConstraint itemConstraint = solver.makeConstraint(Double.NEGATIVE_INFINITY, 0, "item_" + itemId);

            int quantity = itens.getOrDefault(itemId, 0);
            if (quantity > 0) {
                itemConstraint.setCoefficient(p[itemId], quantity);
            }

            for (int aisleIdx = 0; aisleIdx < nAisles; aisleIdx++) {
                int aisleQuantity = aisles.get(aisleIdx).getOrDefault(itemId, 0);
                if (aisleQuantity > 0) {
                    itemConstraint.setCoefficient(c[aisleIdx], -aisleQuantity);
                }
            }
        }

        // Criar grafo de co-ocorrência dos itens
        Map<Integer, Set<Integer>> itemGraph = new HashMap<>();

        for (Map<Integer, Integer> order : orders) {
            List<Integer> itemIds = new ArrayList<>(order.keySet());
            for (int i = 0; i < itemIds.size(); i++) {
                for (int j = i + 1; j < itemIds.size(); j++) {
                    int a = itemIds.get(i);
                    int b = itemIds.get(j);

                    itemGraph.computeIfAbsent(a, k -> new HashSet<>()).add(b);
                    itemGraph.computeIfAbsent(b, k -> new HashSet<>()).add(a);
                }
            }
        }

        List<Set<Integer>> components = new ArrayList<>();
        Set<Integer> visited = new HashSet<>();

        for (int item : itemGraph.keySet()) {
            if (!visited.contains(item)) {
                Set<Integer> component = new HashSet<>();
                dfs(item, itemGraph, visited, component);
                components.add(component);
            }
        }

        for (Set<Integer> group : components) {
            List<Integer> ids = new ArrayList<>(group);
            for (int i = 1; i < ids.size(); i++) {
                MPConstraint cst = solver.makeConstraint(0, 0);
                cst.setCoefficient(p[ids.get(0)], 1);
                cst.setCoefficient(p[ids.get(i)], -1);
            }
        }

        // Função objetivo
        double K = waveSizeLB;
        MPObjective objective = solver.objective();

        for (int i = 0; i < nAisles; i++) {
            objective.setCoefficient(c[i], K);
        }

        for (int i = 0; i < nItems; i++) {
            objective.setCoefficient(p[i], -1 * itens.getOrDefault(i, 0));
        }

        objective.setMinimization();

        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // ms

        solver.setNumThreads(6);

        MPSolver.ResultStatus status = solver.solve();

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {

            Set<Integer> selectedItems = new HashSet<>();
for (int i = 0; i < p.length; i++) {
    if (p[i].solutionValue() == 1.0) {
        selectedItems.add(i);
    }
}

// Agora, para cada pedido, verificar se ele contém algum item selecionado
Set<Integer> selectedOrders = new HashSet<>();
for (int orderId = 0; orderId < orders.size(); orderId++) {
    Map<Integer, Integer> order = orders.get(orderId);
    for (Integer itemId : order.keySet()) {
        if (selectedItems.contains(itemId)) {
            selectedOrders.add(orderId);
            break; // Já encontrou um item selecionado nesse pedido
        }
    }
}

// Para corredores, extrair normalmente
Set<Integer> accessedAisles = new HashSet<>();
for (int i = 0; i < c.length; i++) {
    if (c[i].solutionValue() == 1.0) {
        accessedAisles.add(i);
    }
}

return new ChallengeSolution(selectedOrders, accessedAisles);

        }

        return null;
   }

    private void dfs(int node, Map<Integer, Set<Integer>> graph, Set<Integer> visited, Set<Integer> component) {
        visited.add(node);
        component.add(node);
        for (int neighbor : graph.getOrDefault(node, Set.of())) {
            if (!visited.contains(neighbor)) {
                dfs(neighbor, graph, visited, component);
            }
        }
    }

    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(
                TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
                0);
    }

    protected boolean isSolutionFeasible(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return false;
        }

        int[] totalUnitsPicked = new int[nItems];
        int[] totalUnitsAvailable = new int[nItems];

        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnitsPicked[entry.getKey()] += entry.getValue();
            }
        }

        for (int aisle : visitedAisles) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }

        int totalUnits = Arrays.stream(totalUnitsPicked).sum();
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
            return false;
        }

        for (int i = 0; i < nItems; i++) {
            if (totalUnitsPicked[i] > totalUnitsAvailable[i]) {
                return false;
            }
        }

        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();
        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return 0.0;
        }

        int totalUnitsPicked = 0;

        for (int order : selectedOrders) {
            totalUnitsPicked += orders.get(order).values().stream().mapToInt(Integer::intValue).sum();
        }

        int numVisitedAisles = visitedAisles.size();

        return (double) totalUnitsPicked / numVisitedAisles;
    }
}

