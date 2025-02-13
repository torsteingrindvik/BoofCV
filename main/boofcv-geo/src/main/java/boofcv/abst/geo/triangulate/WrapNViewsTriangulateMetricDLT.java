/*
 * Copyright (c) 2024, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv.abst.geo.triangulate;

import boofcv.abst.geo.Triangulate2ViewsMetric;
import boofcv.abst.geo.TriangulateNViewsMetric;
import boofcv.alg.geo.GeometricResult;
import boofcv.alg.geo.PerspectiveOps;
import boofcv.alg.geo.triangulate.TriangulateMetricLinearDLT;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Point4D_F64;
import georegression.struct.se.Se3_F64;
import lombok.Getter;
import org.ejml.UtilEjml;

import java.util.List;

/**
 * Wrapper around {@link TriangulateMetricLinearDLT} for {@link Triangulate2ViewsMetric}.
 *
 * @author Peter Abeles
 */
public class WrapNViewsTriangulateMetricDLT implements TriangulateNViewsMetric {

	final @Getter TriangulateMetricLinearDLT algorithm = new TriangulateMetricLinearDLT();

	final @Getter Point4D_F64 pointH = new Point4D_F64();

	@Override
	public boolean triangulate( List<Point2D_F64> observations, List<Se3_F64> listWorldToView,
								Point3D_F64 location ) {
		if (GeometricResult.SUCCESS != algorithm.triangulate(observations, listWorldToView, pointH)) {
			return false;
		}

		PerspectiveOps.homogeneousTo3dPositiveZ(pointH, 1e20, UtilEjml.EPS, location);
		return true;
	}
}
