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

package boofcv.alg.geo;

import boofcv.abst.geo.Triangulate2ViewsMetricH;
import boofcv.factory.geo.ConfigTriangulation;
import boofcv.factory.geo.FactoryMultiView;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point4D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

/**
 * <p>
 * Given two views of the same point and a known 3D transform checks to see if the point is in front
 * of both cameras. This is the positive depth constraint. A class is provided instead of a function
 * to reduce computational overhead each time the function is called. Memory only needs to be
 * declared once. Also less chance of messing up and only checking one view instead of two views
 * if you use this class.
 * </p>
 *
 * <p>Triangulation is done in homogeneous coordinates so that points at infinity can be handled</p>
 *
 * <p>
 * COORDINATE SYSTEM: Right handed coordinate system with +z is pointing along the camera's optical axis,
 * </p>
 *
 * @author Peter Abeles
 */
public class PositiveDepthConstraintCheckH {
	// algorithm used to triangulate point location
	Triangulate2ViewsMetricH triangulate;

	// location of triangulated point in 3D space
	Point4D_F64 p = new Point4D_F64();

	public PositiveDepthConstraintCheckH( Triangulate2ViewsMetricH triangulate ) {
		this.triangulate = triangulate;
	}

	public PositiveDepthConstraintCheckH() {
		this(FactoryMultiView.triangulate2ViewMetricH(new ConfigTriangulation(ConfigTriangulation.Type.GEOMETRIC)));
	}

	/**
	 * Checks to see if a single point meets the constraint.
	 *
	 * @param viewA View of the 3D point from the first camera. Normalized image coordinates.
	 * @param viewB View of the 3D point from the second camera. Normalized image coordinates.
	 * @param fromAtoB Transform from the B to A camera frame.
	 * @return If the triangulated point appears in front of both cameras.
	 */
	public boolean checkConstraint( Point2D_F64 viewA, Point2D_F64 viewB, Se3_F64 fromAtoB ) {
		if (!triangulate.triangulate(viewA, viewB, fromAtoB, p))
			throw new RuntimeException("Triangulate failed. p1=" + viewA + " p2=" + viewB);

		if (PerspectiveOps.isBehindCamera(p))
			return false;

		SePointOps_F64.transform(fromAtoB, p, p);

		return !PerspectiveOps.isBehindCamera(p);
	}
}
