package it.units.erallab.robotevo2d.main.singleagent;

import it.units.erallab.mrsim2d.builder.Param;
import it.units.erallab.mrsim2d.builder.ParamMap;
import it.units.erallab.mrsim2d.core.tasks.Task;

/**
 * @author "Eric Medvet" on 2022/09/01 for 2d-robot-evolution
 */
public record NamedTask<Q>(
    @Param("task") Task<Q, ?> task,
    @Param(value = "", self = true) ParamMap map
) {}