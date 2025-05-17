package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import com.google.ortools.Loader;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPVariable;

public class ChallengeSolver {
    private final long MAX_RUNTIME = 600000; // 10 minutes in milliseconds

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
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

    protected List<Map.Entry<Integer, Integer>> getSortedItemsFromOrders(List<Map<Integer, Integer>> orders) {
        Map<Integer, Integer> totalItensMap = new java.util.HashMap<>();

        for (Map<Integer, Integer> order : orders) {
            for (Map.Entry<Integer, Integer> entry : order.entrySet()) {
                int itemId = entry.getKey();
                int quantity = entry.getValue();
                totalItensMap.put(itemId, totalItensMap.getOrDefault(itemId, 0) + quantity);
            }
        }

        List<Map.Entry<Integer, Integer>> sortedItems = new ArrayList<>(totalItensMap.entrySet());
        sortedItems.sort((a, b) -> Integer.compare(b.getValue(), a.getValue()));
        return sortedItems;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        MPSolver solver = MPSolver.createSolver("SCIP");

        if (solver == null) {
            System.out.println("Could not create solver SCIP");
            return new ChallengeSolution(Set.of(), Set.of());
        }

        this.nOrders = orders.size();
        this.nAisles = aisles.size();

        List<Map.Entry<Integer, Integer>> sortedItems = getSortedItemsFromOrders(orders);
        getItemsPerOrder(orders);

        MPVariable[] p = new MPVariable[nOrders];
        MPVariable[] c = new MPVariable[nAisles];

        for (int i = 0; i < nOrders; i++) {
            p[i] = solver.makeBoolVar("p_" + i);
        }
        for (int i = 0; i < nAisles; i++) {
            c[i] = solver.makeBoolVar("c_" + i);
        }

        // Restrição de tamanho da wave
        MPConstraint itemsBound = solver.makeConstraint(waveSizeLB, waveSizeUB, "waveSize");
        for (int i = 0; i < nOrders; i++) {
            itemsBound.setCoefficient(p[i], nItemsPerOrder[i]);
        }

        // Restrições de disponibilidade de itens
        for (int itemId = 0; itemId < nItems; itemId++) {
            MPConstraint itemConstraint = solver.makeConstraint(Double.NEGATIVE_INFINITY, 0, "item_" + itemId);

            for (int orderIdx = 0; orderIdx < nOrders; orderIdx++) {
                int quantity = orders.get(orderIdx).getOrDefault(itemId, 0);
                if (quantity > 0) {
                    itemConstraint.setCoefficient(p[orderIdx], quantity);
                }
            }

            for (int aisleIdx = 0; aisleIdx < nAisles; aisleIdx++) {
                int quantity = aisles.get(aisleIdx).getOrDefault(itemId, 0);
                if (quantity > 0) {
                    itemConstraint.setCoefficient(c[aisleIdx], -quantity);
                }
            }
        }

        // Seleciona o item mais comum
        Map.Entry<Integer, Integer> primeiroItem = sortedItems.get(0);
        int itemId = primeiroItem.getKey();

        // Encontra prateleira com maior quantidade do item
        List<Map.Entry<Integer, Integer>> prateleirasComItem = new ArrayList<>();
        for (int i = 0; i < aisles.size(); i++) {
            int quantidade = aisles.get(i).getOrDefault(itemId, 0);
            if (quantidade > 0) {
                prateleirasComItem.add(Map.entry(i, quantidade));
            }
        }

        prateleirasComItem.sort((a, b) -> Integer.compare(b.getValue(), a.getValue()));
        Map.Entry<Integer, Integer> melhorPrateleira = prateleirasComItem.get(0);
        int ptI = melhorPrateleira.getKey();

        // Objective: minimizar corredores acessados e maximizar itens coletados
        double M = 100000.0;
        double K = 1.0;
        MPObjective objective = solver.objective();

        for (int i = 0; i < nAisles; i++) {
            if (i == ptI) {
                objective.setCoefficient(c[i], M * 10); // mais caro abrir essa prateleira
            } else {
                objective.setCoefficient(c[i], M);
            }
        }

        for (int orderIdx = 0; orderIdx < nOrders; orderIdx++) {
            long totalItemsInOrder = nItemsPerOrder[orderIdx];
            objective.setCoefficient(p[orderIdx], -K * totalItemsInOrder);
        }

        objective.setMinimization();

        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // milliseconds
        solver.setNumThreads(6); // ajuste conforme necessário

        MPSolver.ResultStatus status = solver.solve();

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            Set<Integer> selectedOrders = new HashSet<>();
            Set<Integer> accessedAisles = new HashSet<>();

            for (int i = 0; i < p.length; i++) {
                if (p[i].solutionValue() == 1.0) {
                    selectedOrders.add(i);
                }
            }
            for (int i = 0; i < c.length; i++) {
                if (c[i].solutionValue() == 1.0) {
                    accessedAisles.add(i);
                }
            }

            return new ChallengeSolution(selectedOrders, accessedAisles);
        }

        return new ChallengeSolution(Set.of(), Set.of());
    }

    protected long getRemainingTime(StopWatch stopWatch) {
        return Math.max(
                TimeUnit.SECONDS.convert(MAX_RUNTIME - stopWatch.getTime(TimeUnit.MILLISECONDS), TimeUnit.MILLISECONDS),
                0
        );
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
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) return false;

        for (int i = 0; i < nItems; i++) {
            if (totalUnitsPicked[i] > totalUnitsAvailable[i]) return false;
        }

        return true;
    }

    protected double computeObjectiveFunction(ChallengeSolution challengeSolution) {
        Set<Integer> selectedOrders = challengeSolution.orders();
        Set<Integer> visitedAisles = challengeSolution.aisles();

        if (selectedOrders == null || visitedAisles == null || selectedOrders.isEmpty() || visitedAisles.isEmpty()) {
            return 0.0;
        }

        int totalUnitsPicked = selectedOrders.stream()
                .flatMapToInt(order -> orders.get(order).values().stream().mapToInt(Integer::intValue))
                .sum();

        int numVisitedAisles = visitedAisles.size();
        return (double) totalUnitsPicked / numVisitedAisles;
    }
}

