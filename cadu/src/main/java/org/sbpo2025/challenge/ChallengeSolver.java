package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

// import java.util.ArrayList;
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
    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;
    
    private int nOrders;
    private int nAisles;
    private long[] nItemsPerOrder;

    static { Loader.loadNativeLibraries(); }

    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    protected void getItemsPerOrder(List<Map<Integer, Integer>> orders) {
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

    public ChallengeSolution solve(StopWatch stopWatch) {
        MPSolver solver = MPSolver.createSolver("SCIP");
        
        if (solver == null)
            return null;

        this.nOrders = orders.size(); // Numero de pedidos
        this.nAisles = aisles.size(); // Numero de corredores
        
        getItemsPerOrder(orders);

        // Criação das boolenas de corredores(c) e pedidos(p)
        MPVariable[] p = new MPVariable[nOrders];
        MPVariable[] c = new MPVariable[nAisles];

        for (int i = 0; i < nOrders; i++) {
            p[i] = solver.makeBoolVar("p_" + i);
        }
        for (int i = 0; i < nAisles; i++) {
            c[i] = solver.makeBoolVar("c_" + i);
        }

        // Restrição de tamanho da wave (LB e UB)
        MPConstraint itemsBound = solver.makeConstraint(Math.max(waveSizeLB, 1), waveSizeUB, "waveSize");
        for (int i = 0; i < nOrders; i++) {
            itemsBound.setCoefficient(p[i], nItemsPerOrder[i]);
        }

        // Restrições de disponibilidade de itens
        for (int itemId = 0; itemId < nItems; itemId++) {
            MPConstraint itemConstraint = solver.makeConstraint(
                Double.NEGATIVE_INFINITY, 0, "item_" + itemId);
            
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

        // Objective 
        // Minimize (10^5*Corredores - K*Itens)
        
        double M = 100000.0; // Peso corredores (ajustar)
        double K = 1.0; // Peso itens (ajustar)
        MPObjective objective = solver.objective();

        for (int i = 0; i < nAisles; i++) {
            objective.setCoefficient(c[i], M);
        }

        for (int orderIdx = 0; orderIdx < nOrders; orderIdx++) {
            long totalItemsInOrder = 0;
            for (int quantity : orders.get(orderIdx).values()) {
                totalItemsInOrder += quantity;
            }
            objective.setCoefficient(p[orderIdx], -K * totalItemsInOrder);
        }

        objective.setMinimization();

        // Indicando o time limit
        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // Convert to milliseconds
        
        // Habilitante o uso de múltiplos threads
        solver.setNumThreads(6); // Ajustar o numero de threads (oficial é 8)

        // Solve
        MPSolver.ResultStatus status = solver.solve();

        // Extraindo a solução
        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            System.out.println("Solver found a solution: " + status);

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

        return null;
    }

    /*
     * Get the remaining time in seconds
     */
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

        // Calculate total units picked
        for (int order : selectedOrders) {
            for (Map.Entry<Integer, Integer> entry : orders.get(order).entrySet()) {
                totalUnitsPicked[entry.getKey()] += entry.getValue();
            }
        }

        // Calculate total units available
        for (int aisle : visitedAisles) {
            for (Map.Entry<Integer, Integer> entry : aisles.get(aisle).entrySet()) {
                totalUnitsAvailable[entry.getKey()] += entry.getValue();
            }
        }

        // Check if the total units picked are within bounds
        int totalUnits = Arrays.stream(totalUnitsPicked).sum();
        if (totalUnits < waveSizeLB || totalUnits > waveSizeUB) {
            return false;
        }

        // Check if the units picked do not exceed the units available
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

        // Calculate total units picked
        for (int order : selectedOrders) {
            totalUnitsPicked += orders.get(order).values().stream()
                    .mapToInt(Integer::intValue)
                    .sum();
        }

        // Calculate the number of visited aisles
        int numVisitedAisles = visitedAisles.size();

        // Objective function: total units picked / number of visited aisles
        return (double) totalUnitsPicked / numVisitedAisles;
    }
}
