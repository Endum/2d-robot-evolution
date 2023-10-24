package io.github.ericmedvet.robotevo2d.main.builders;

import io.github.ericmedvet.jnb.core.Discoverable;
import io.github.ericmedvet.jnb.core.Param;
import io.github.ericmedvet.jsdynsym.core.numerical.MultivariateRealFunction;
import io.github.ericmedvet.jsdynsym.core.numerical.NumericalDynamicalSystem;
import io.github.ericmedvet.mrsim2d.core.Sensor;
import io.github.ericmedvet.mrsim2d.core.bodies.Voxel;
import io.github.ericmedvet.robotevo2d.main.agents.selfassemblyvsr.NumSelfAssemblyVSR;
import io.github.ericmedvet.jsdynsym.buildable.builders.NumericalDynamicalSystems;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Stream;

@Discoverable(prefixTemplate = "sim|s.agent|a")
public class Agents {

    private Agents() {
    }

    @SuppressWarnings({"unused", "unchecked"})
    public static NumSelfAssemblyVSR numSelfAssemblyVSR(
            @Param("unitNumber") int unitNumber,
            @Param(value = "voxelSideLength", dD = 1.0) double voxelSideLength,
            @Param(value = "voxelMass", dD = 1.0) double voxelMass,
            @Param(value = "nSignals", dI = 1) int nSignals,
            @Param("function") NumericalDynamicalSystems.Builder<?, ?> numericalDynamicalSystemBuilder,
            @Param("sensors") List<Sensor<? super Voxel>> sensors) {
        return new NumSelfAssemblyVSR(
                unitNumber,
                new Voxel.Material(),
                voxelSideLength,
                voxelMass,
                nSignals,
                (List<NumericalDynamicalSystem<?>>) Stream.generate( () ->
                    numericalDynamicalSystemBuilder.apply(
                        MultivariateRealFunction.varNames(
                            "x",
                            NumSelfAssemblyVSR.nOfInputs(sensors.size(), nSignals)),
                        MultivariateRealFunction.varNames(
                            "y",
                            NumSelfAssemblyVSR.nOfOutputs(nSignals)))
                ).limit(unitNumber).toList(),
                Stream.generate(() -> List.copyOf(sensors)).limit(unitNumber).toList()
        );
    }
}
