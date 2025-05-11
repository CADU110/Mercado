package org.sbpo2025.challenge;

import org.apache.commons.lang3.time.StopWatch;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import com.google.ortools.sat.CpModel;
import com.google.ortools.sat.CpSolver;
import com.google.ortools.sat.CpSolverStatus;
import com.google.ortools.sat.IntVar;
import com.google.ortools.sat.LinearExpr;
import com.google.ortools.sat.BoolVar;

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

    // Essa parte é para carregar o OR-TOOLS manualmente, caso não consiga
//    static {
//        try {
//            System.load("$HOME/.m2/repository/com/google/ortools/ortools-linux-x86-64/9.11.4210/ortools-linux-x86-64/libjniortools.so");
//        } catch (UnsatisfiedLinkError e) {
//            System.err.println("Native code library failed to load.\n" + e);
//            System.exit(1);
//        }
//    }
    
    public ChallengeSolver(
            List<Map<Integer, Integer>> orders, List<Map<Integer, Integer>> aisles, int nItems, int waveSizeLB, int waveSizeUB) {
        this.orders = orders;
        this.aisles = aisles;
        this.nItems = nItems;
        this.waveSizeLB = waveSizeLB;
        this.waveSizeUB = waveSizeUB;
    }

    protected void getItemsPerOrder(List<Map<Integer, Integer>> orders){
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
        // Inicialização
        CpModel model = new CpModel();

        this.nOrders = orders.size(); // número de pedidos
        this.nAisles = aisles.size(); // número de corredores

        getItemsPerOrder(orders); // atualiza o vetor que mantém a informação

        BoolVar[] p = new BoolVar[nOrders];
        BoolVar[] c = new BoolVar[nAisles];

        // Criação das booleanas p (pedidos) e c (corredores)
        for (int i = 0; i < nOrders; i++)
            p[i] = model.newBoolVar("p_"+i);
        for (int i = 0; i < nAisles; i++)
            c[i] = model.newBoolVar("c_"+i);        

        // Restrições de LB e UB
        LinearExpr totalItemsSelectedExpression = LinearExpr.weightedSum(p, nItemsPerOrder);
        LinearExpr totalAislesSelected = LinearExpr.sum(c);

        model.addLessOrEqual(
            totalItemsSelectedExpression,
            waveSizeUB
        );

        model.addGreaterOrEqual(
            totalItemsSelectedExpression,
            waveSizeLB
        );

        // Restrições de item
        for (int itemId = 0; itemId < nItems; itemId++) {
            List<BoolVar> pVars = new ArrayList<>();
            List<Long> pCoeffs = new ArrayList<>();
            List<BoolVar> cVars = new ArrayList<>();
            List<Long> cCoeffs = new ArrayList<>();
            
            for (int orderIdx = 0; orderIdx < nOrders; orderIdx++) {
                int quantity = orders.get(orderIdx).getOrDefault(itemId, 0);
                if (quantity > 0) {
                    pVars.add(p[orderIdx]);
                    pCoeffs.add((long)quantity);
                }
            }
            
            for (int aisleIdx = 0; aisleIdx < nAisles; aisleIdx++) {
                int quantity = aisles.get(aisleIdx).getOrDefault(itemId, 0);
                if (quantity > 0) {
                    cVars.add(c[aisleIdx]);
                    cCoeffs.add((long)quantity);
                }
            }
            
            if (!pVars.isEmpty() && !cVars.isEmpty()) {
                model.addLessOrEqual(
                    LinearExpr.weightedSum(pVars.toArray(new BoolVar[0]), 
                    pCoeffs.stream().mapToLong(Long::longValue).toArray()),
                    LinearExpr.weightedSum(cVars.toArray(new BoolVar[0]),
                    cCoeffs.stream().mapToLong(Long::longValue).toArray())
                );
            }
        }

        // Objetivo

        // Para o teste ficou
        // minimize (total_aisles - K*total_items)
        
        double K = 10.0; // peso
        double scalingFactor = 1000.0; // escala

        long scaledWeight = (long)(-K * scalingFactor);

        if (scaledWeight < Integer.MIN_VALUE || scaledWeight > Integer.MAX_VALUE) {
            throw new ArithmeticException("Scaling factor too large");
        }

        model.minimize(
            LinearExpr.sum(new LinearExpr[] {
                totalAislesSelected,
                LinearExpr.term(totalItemsSelectedExpression, (long)(scaledWeight))
            })
        );


        // Solução
        CpSolver solver = new CpSolver();
        CpSolverStatus status = solver.solve(model);

        // Extração da solução
        if (status == CpSolverStatus.OPTIMAL || status == CpSolverStatus.FEASIBLE) {
            Set<Integer> selectedOrders = new HashSet<>();
            Set<Integer> accessedAisles = new HashSet<>();
            
            for (int i = 0; i < p.length; i++) {
                if (solver.value(p[i]) == 1L) {
                    selectedOrders.add(i);
                }
            }
            for (int i = 0; i < c.length; i++) {
                if (solver.value(c[i]) == 1L) {
                    accessedAisles.add(i);
                }
            }
            return new ChallengeSolution(selectedOrders, accessedAisles);
        }
        return new ChallengeSolution(Set.of(), Set.of());
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
