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
    private long[] itemsOrdered;

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
        this.itemsOrdered = new long[this.nItems];
        
        for (int i = 0; i < this.nOrders; i++) {
            Map<Integer, Integer> order = orders.get(i);
            long totalItems = 0;
            
            for (int item : order.keySet()) {
                totalItems += order.get(item);
                this.itemsOrdered[item] += order.get(item);
            }
            
            this.nItemsPerOrder[i] = totalItems;
        }
    }

    private  ChallengeSolution optimalResult(StopWatch stopWatch, MPSolver solver, MPVariable[] p, MPVariable[] c, double partialScore, int nAislesAccessed) {
        MPConstraint minCorridors = solver.makeConstraint(nAislesAccessed+1, Double.POSITIVE_INFINITY, "minCorridors");
        for (int i = 0; i < nAisles; i++) {
            minCorridors.setCoefficient(c[i], 1);
        }

        double newScore = 0.0;
        double bestScore = partialScore;

        ChallengeSolution partialSolution = null;
        ChallengeSolution bestSolution = null;


        do {
            long remainingTime = getRemainingTime(stopWatch) - 5;
            solver.setTimeLimit(remainingTime * 1000); // millisegundos

            System.out.println("Solving with aisles ≥ " + minCorridors.lb());
            MPSolver.ResultStatus status = solver.solve();

            if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
                Set<Integer> selectedOrders = new HashSet<>();
                Set<Integer> accessedAisles = new HashSet<>();

                int totalItemsPickedUp = 0;
                int totalAislesAccessed = 0;
                
                for (int i = 0; i < p.length; i++) {
                    if (p[i].solutionValue() == 1.0) {
                        totalItemsPickedUp += nItemsPerOrder[i];
                        selectedOrders.add(i);
                    }
                }
                for (int i = 0; i < c.length; i++) {
                    if (c[i].solutionValue() == 1.0) {
                        totalAislesAccessed += 1;
                        accessedAisles.add(i);
                    }
                }
                
                partialSolution = new ChallengeSolution(selectedOrders, accessedAisles);
                newScore = computeObjectiveFunction(partialSolution);

                if (!isSolutionFeasible(partialSolution)) {
                    System.out.println("Solution is not feasible");
                    return null;
                }

                if (newScore > bestScore) {
                    bestScore = newScore;
                    bestSolution = partialSolution;
                }

                System.out.println("Solver found a new solution: " + status);
                System.out.println("Objective function value: " + computeObjectiveFunction(partialSolution));
                System.out.println("Duration: " + (remainingTime + 5 - getRemainingTime(stopWatch)) + "s");
                System.out.println("Remaining time: " + getRemainingTime(stopWatch) + "s");
                System.out.println("Total items picked up: " + totalItemsPickedUp + " (out of " + waveSizeUB + ")");
                System.out.println("Total aisles accessed: " + totalAislesAccessed + "\n");

                if (totalAislesAccessed+1 <= nAisles) minCorridors.setLb(totalAislesAccessed+1);
                else break;
            }
            else {
                System.out.println("Solver found no solution: " + status + "\n");
                break;
            }
        } while (getRemainingTime(stopWatch) > 10 && newScore > 0.0);

        System.out.println("Best solution found: " + bestScore);
        return bestSolution;
    }

    private ChallengeSolution feasibleResult(StopWatch stopWatch, MPSolver solver, MPVariable[] p, MPVariable[] c, double partialScore) {
        // Re-fazendo o objetivo
        MPObjective objective = solver.objective();

        // objective.clear();

        double M = partialScore*1.15; // Base penalty for using a corridor
        // double K = 1.0;      // Base gain per item in selected orders

        for (int i = 0; i < nOrders; i++) {
            long totalItemsInOrder = 0;
            for (int quantity : orders.get(i).values()) {
                totalItemsInOrder += quantity;
            }
            objective.setCoefficient(p[i], -totalItemsInOrder);
        }

        for (int i = 0; i < nAisles; i++) {
            objective.setCoefficient(c[i], M);
        }

        objective.setMinimization();

        double newScore = 0.0;
        double bestScore = partialScore;

        ChallengeSolution partialSolution = null;
        ChallengeSolution bestSolution = null;

        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // millisegundos

        System.out.println("Solving with M = " + partialScore*1.15);
        MPSolver.ResultStatus status = solver.solve();

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            Set<Integer> selectedOrders = new HashSet<>();
            Set<Integer> accessedAisles = new HashSet<>();

            int totalItemsPickedUp = 0;
            int totalAislesAccessed = 0;
            
            for (int i = 0; i < p.length; i++) {
                if (p[i].solutionValue() == 1.0) {
                    totalItemsPickedUp += nItemsPerOrder[i];
                    selectedOrders.add(i);
                }
            }
            for (int i = 0; i < c.length; i++) {
                if (c[i].solutionValue() == 1.0) {
                    totalAislesAccessed += 1;
                    accessedAisles.add(i);
                }
            }
            
            partialSolution = new ChallengeSolution(selectedOrders, accessedAisles);
            newScore = computeObjectiveFunction(partialSolution);

            if (!isSolutionFeasible(partialSolution)) {
                System.out.println("Solution is not feasible");
                return null;
            }

            if (newScore > bestScore) {
                bestScore = newScore;
                bestSolution = partialSolution;
            }

            System.out.println("Solver found a new solution: " + status);
            System.out.println("Objective function value: " + computeObjectiveFunction(partialSolution));
            System.out.println("Duration: " + (remainingTime + 5 - getRemainingTime(stopWatch)) + "s");
            System.out.println("Remaining time: " + getRemainingTime(stopWatch) + "s");
            System.out.println("Total items picked up: " + totalItemsPickedUp + " (out of " + waveSizeUB + ")");
            System.out.println("Total aisles accessed: " + totalAislesAccessed);
        }

        return bestSolution;
    }

    private ChallengeSolution notFoundResult(StopWatch stopWatch, MPSolver solver, MPObjective objective, MPVariable[] p, MPVariable[] c) {
        // Re-fazendo o objetivo
        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // millisegundos

        MPSolver.ResultStatus status = solver.solve();

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            Set<Integer> selectedOrders = new HashSet<>();
            Set<Integer> accessedAisles = new HashSet<>();

            int totalItemsPickedUp = 0;
            int totalAislesAccessed = 0;
            
            for (int i = 0; i < p.length; i++) {
                if (p[i].solutionValue() == 1.0) {
                    totalItemsPickedUp += nItemsPerOrder[i];
                    selectedOrders.add(i);
                }
            }
            for (int i = 0; i < c.length; i++) {
                if (c[i].solutionValue() == 1.0) {
                    totalAislesAccessed += 1;
                    accessedAisles.add(i);
                }
            }
            
            ChallengeSolution challengeSolution = new ChallengeSolution(selectedOrders, accessedAisles);
            double score = computeObjectiveFunction(challengeSolution);

            if (!isSolutionFeasible(challengeSolution)) {
                System.out.println("Solution is not feasible");
                return null;
            }

            System.out.println("Solver found a new solution: " + status);
            System.out.println("Objective function value: " + score);
            System.out.println("Duration: " + (remainingTime + 5 - getRemainingTime(stopWatch)) + "s");
            System.out.println("Remaining time: " + getRemainingTime(stopWatch) + "s");
            System.out.println("Total items picked up: " + totalItemsPickedUp + " (out of " + waveSizeUB + ")");
            System.out.println("Total aisles accessed: " + totalAislesAccessed);
            
            return challengeSolution;
        }
        
        return null;
    }


    public ChallengeSolution solve(StopWatch stopWatch) {
        MPSolver solver = MPSolver.createSolver("SAT");
        
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

        // Objetivo
        // Min (UB*Corredores - 1*Itens)
        
        // Constants to tune the relative importance of corridors vs. items
        double M = 100000; // Base penalty for using a corridor
        // double K = 1.0;      // Base gain per item in selected orders

        // Define the objective function
        MPObjective objective = solver.objective();


        for (int i = 0; i < nAisles; i++) {
            objective.setCoefficient(c[i], M);
        }

        // Reward orders based on the number of items they contribute
        for (int orderIdx = 0; orderIdx < nOrders; orderIdx++) {
            long totalItemsInOrder = 0;
            for (int quantity : orders.get(orderIdx).values()) {
                totalItemsInOrder += quantity;
            }
            objective.setCoefficient(p[orderIdx], -totalItemsInOrder);
        }

        // Set the objective to minimization
        objective.setMinimization();

        //  Indica o limite de tempo
        solver.setTimeLimit(90 * 1000); // 90 segundos (em ms)
        
        // Habilita o uso de múltiplos threads
        solver.setNumThreads(6); // Ajustar para a maquina em que vais rodar (oficial é 8)

        // Solve

        long remainingTime = getRemainingTime(stopWatch);

        MPSolver.ResultStatus status = solver.solve();

        // Extraindo a solução
        ChallengeSolution partialSolution = null;
        double partialScore = 0.0;

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            System.out.println("Solver found a partial solution: " + status);

            Set<Integer> selectedOrders = new HashSet<>();
            Set<Integer> accessedAisles = new HashSet<>();

            int totalItemsPickedUp = 0;
            int totalAislesAccessed = 0;
            
            for (int i = 0; i < p.length; i++) {
                if (p[i].solutionValue() == 1.0) {
                    totalItemsPickedUp += nItemsPerOrder[i];
                    selectedOrders.add(i);
                }
            }
            for (int i = 0; i < c.length; i++) {
                if (c[i].solutionValue() == 1.0) {
                    totalAislesAccessed += 1;
                    accessedAisles.add(i);
                }
            }
            
            partialSolution = new ChallengeSolution(selectedOrders, accessedAisles);
            partialScore = computeObjectiveFunction(partialSolution);
            
            if (!isSolutionFeasible(partialSolution)) {
                System.out.println("Solution is not feasible");
                return null;
            }

            
            System.out.println("Objective function value: " + partialScore);
            System.out.println("Duration: " + (remainingTime - getRemainingTime(stopWatch)) + "s");
            System.out.println("Remaining time: " + getRemainingTime(stopWatch) + "s");
            System.out.println("Total items picked up: " + totalItemsPickedUp + " (out of " + waveSizeUB + ")");
            System.out.println("Total aisles accessed: " + totalAislesAccessed + "\n");

            // System.out.println("Score com 10^5" + (100000 * totalAislesAccessed - totalItemsPickedUp));
            // System.out.println("Score com UB: " + (waveSizeUB * totalAislesAccessed - totalItemsPickedUp));
            
            ChallengeSolution newSolution = null;

            if (status == MPSolver.ResultStatus.OPTIMAL) {
                newSolution = optimalResult(stopWatch, solver, p, c, partialScore, totalAislesAccessed);
            }
            else if (status == MPSolver.ResultStatus.FEASIBLE) {
                newSolution = feasibleResult(stopWatch, solver, p, c, partialScore);
            }

            if (newSolution != null && computeObjectiveFunction(newSolution) > partialScore) {
                return newSolution;
            } else {
                return partialSolution;
            }
        }
        else {
            return notFoundResult(stopWatch, solver, objective, p, c);
        }

        // return null;
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
