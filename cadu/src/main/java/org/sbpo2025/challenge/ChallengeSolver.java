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
    static { Loader.loadNativeLibraries(); }

    private final long MAX_RUNTIME = 600000; // milliseconds; 10 minutes

    protected List<Map<Integer, Integer>> orders;
    protected List<Map<Integer, Integer>> aisles;
    protected int nItems;
    protected int waveSizeLB;
    protected int waveSizeUB;
    
    private int nOrders;
    private int nAisles;
    private int[] nItemsPerOrder;

    private MPVariable[] o;
    private MPVariable[] a;
    private MPSolver solver;

    public ChallengeSolver(List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;

        this.nOrders = orders.size();
        this.nAisles = aisles.size();
        getItemsPerOrder(orders);

        this.o = new MPVariable[nOrders];
        this.a = new MPVariable[nAisles];
    }

    protected void getItemsPerOrder(List<Map<Integer, Integer>> orders) {
        this.nItemsPerOrder = new int[this.nOrders];
        
        for (int i = 0; i < this.nOrders; i++) {
            Map<Integer, Integer> order = orders.get(i);
            int totalItems = 0;
            
            for (int item : order.keySet()) {
                totalItems += order.get(item);
            }
            
            this.nItemsPerOrder[i] = totalItems;
        }
    }

    private  ChallengeSolution optimalResult(StopWatch stopWatch, double partialScore, int nAislesAccessed) {
        // Cria nova restrição para o número de corredores
        MPConstraint minCorridors = solver.makeConstraint(nAislesAccessed+1, Double.POSITIVE_INFINITY, "minCorridors");
        for (int i = 0; i < nAisles; i++) {
            minCorridors.setCoefficient(a[i], 1);
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
                System.out.println("Solver found a new solution: " + status);

                partialSolution = extractSolution();

                if (partialSolution == null)
                    return null;

                newScore = computeObjectiveFunction(partialSolution);

                if (newScore > bestScore) {
                    bestScore = newScore;
                    bestSolution = partialSolution;
                }

                // Atualiza o limite inferior do número de corredores
                int totalAislesAccessed = partialSolution.aisles().size();

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

    private ChallengeSolution feasibleResult(StopWatch stopWatch, double partialScore) {
        // Re-fazendo o objetivo
        MPObjective objective = solver.objective();

        double M = partialScore*1.15; // Penalidada ajustada com expectativa de melhora da solução

        for (int i = 0; i < nOrders; i++) {
            long totalItemsInOrder = 0;
            for (int quantity : orders.get(i).values()) {
                totalItemsInOrder += quantity;
            }
            objective.setCoefficient(o[i], -totalItemsInOrder);
        }

        for (int i = 0; i < nAisles; i++) {
            objective.setCoefficient(a[i], M);
        }

        objective.setMinimization();

        // Resolver novamente
        double newScore = 0.0;
        double bestScore = partialScore;

        ChallengeSolution partialSolution = null;
        ChallengeSolution bestSolution = null;

        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // millisegundos

        System.out.println("Solving with M = " + partialScore*1.15);
        MPSolver.ResultStatus status = solver.solve();

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            System.out.println("Solver found a new solution: " + status);

            partialSolution = extractSolution();

            if (partialSolution == null)
                return null;

            newScore = computeObjectiveFunction(partialSolution);

            if (newScore > bestScore) {
                bestScore = newScore;
                bestSolution = partialSolution;
            }
        }

        return bestSolution;
    }

    private ChallengeSolution notFoundResult(StopWatch stopWatch) {
        long remainingTime = getRemainingTime(stopWatch) - 5;
        solver.setTimeLimit(remainingTime * 1000); // millisegundos

        MPSolver.ResultStatus status = solver.solve();

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            System.out.println("Solver found a new solution: " + status);
            
            ChallengeSolution challengeSolution = extractSolution();
            
            return challengeSolution;
        }
        return null;
    }

    private ChallengeSolution extractSolution() {
        Set<Integer> selectedOrders = new HashSet<>();
        Set<Integer> accessedAisles = new HashSet<>();

        int totalItemsPickedUp = 0;
        int totalAislesAccessed = 0;
        
        for (int i = 0; i < nOrders; i++) {
            if (o[i].solutionValue() == 1.0) {
                totalItemsPickedUp += nItemsPerOrder[i];
                selectedOrders.add(i);
            }
        }
        for (int i = 0; i < nAisles; i++) {
            if (a[i].solutionValue() == 1.0) {
                totalAislesAccessed += 1;
                accessedAisles.add(i);
            }
        }

        ChallengeSolution solution = new ChallengeSolution(selectedOrders, accessedAisles);


        if (!isSolutionFeasible(solution)) {
            System.out.println("Solution is not feasible");
            return null;
        }

        System.out.println("Objective function value: " + computeObjectiveFunction(solution));
        System.out.println("Total items picked up: " + totalItemsPickedUp + " (out of " + waveSizeUB + ")");
        System.out.println("Total aisles accessed: " + totalAislesAccessed + "\n");

        return solution;
    }

    public ChallengeSolution solve(StopWatch stopWatch) {
        System.out.println(getRemainingTime(stopWatch) + "s) - Starting solver...");

        this.solver = MPSolver.createSolver("SAT");
        
        if (solver == null)
            return null;

        // Criação das boolenas de corredores(c) e pedidos(p)
        for (int i = 0; i < nOrders; i++) {
            this.o[i] = solver.makeBoolVar("o_" + i);
        }
        for (int i = 0; i < nAisles; i++) {
            this.a[i] = solver.makeBoolVar("a_" + i);
        }

        System.out.println(getRemainingTime(stopWatch) + "s) - Creating bound restrictions...");

        // Restrição de tamanho da wave (LB e UB)
        MPConstraint itemsBound = solver.makeConstraint(Math.max(waveSizeLB, 1), waveSizeUB, "waveSize");
        for (int i = 0; i < nOrders; i++) {
            itemsBound.setCoefficient(o[i], nItemsPerOrder[i]);
        }

        System.out.println(getRemainingTime(stopWatch) + "s) - Creating item restrictions...");

        // Restrições de disponibilidade de itens
        for (int idItem = 0; idItem < nItems; idItem++) {
            MPConstraint itemConstraint = solver.makeConstraint(
                Double.NEGATIVE_INFINITY, 0, "item_" + idItem);
            
            for (int i = 0; i < nOrders; i++) {
                int quantity = orders.get(i).getOrDefault(idItem, 0);
                if (quantity > 0) {
                    itemConstraint.setCoefficient(o[i], quantity);
                }
            }
            for (int i = 0; i < nAisles; i++) {
                int quantity = aisles.get(i).getOrDefault(idItem, 0);
                if (quantity > 0) {
                    itemConstraint.setCoefficient(a[i], -quantity);
                }
            }
        }

        // Objetivo
        // Min (UB*Corredores - 1*Itens)

        System.out.println(getRemainingTime(stopWatch) + "s) - Defining objective...");
        
        // Constants to tune the relative importance of corridors vs. items
        double M = waveSizeUB; // Base penalty for using a corridor

        // Define the objective function
        MPObjective objective = solver.objective(); 

        for (int i = 0; i < nAisles; i++) {
            objective.setCoefficient(a[i], M);
        }

        // Reward orders based on the number of items they contribute
        for (int i = 0; i < nOrders; i++) {
            objective.setCoefficient(o[i], -nItemsPerOrder[i]);
        }

        // Set the objective to minimization
        objective.setMinimization();

        //  Indica o limite de tempo
        solver.setTimeLimit(90 * 1000); // 90 segundos (em ms)
        
        // Habilita o uso de múltiplos threads
        solver.setNumThreads(6); // Ajustar para a maquina em que vais rodar (oficial é 8)

        // Solve
        System.out.println(getRemainingTime(stopWatch) + "s) - Solving...\n");
        MPSolver.ResultStatus status = solver.solve();

        // Extraindo a solução
        ChallengeSolution partialSolution = null;
        double partialScore = 0.0;

        if (status == MPSolver.ResultStatus.OPTIMAL || status == MPSolver.ResultStatus.FEASIBLE) {
            System.out.println("Solver found a partial solution: " + status);
            
            partialSolution = extractSolution();
            partialScore = computeObjectiveFunction(partialSolution);
            
            ChallengeSolution newSolution = null;

            if (status == MPSolver.ResultStatus.OPTIMAL) {
                newSolution = optimalResult(stopWatch, partialScore, partialSolution.aisles().size());
            }
            else if (status == MPSolver.ResultStatus.FEASIBLE) {
                newSolution = feasibleResult(stopWatch, partialScore);
            }

            if (newSolution != null && computeObjectiveFunction(newSolution) > partialScore) {
                return newSolution;
            } else {
                return partialSolution;
            }
        }
        else {
            return notFoundResult(stopWatch);
        }
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
