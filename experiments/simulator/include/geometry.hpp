/*
 * geometry.hpp
 *
 *  Created on: Jun 2, 2018
 *      Author: rob
 */

#ifndef INCLUDE_SIM_GEOMETRY_HPP_
#define INCLUDE_SIM_GEOMETRY_HPP_


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Line_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Object.h>

#include <CGAL/Alpha_shape_3.h>
#include <CGAL/Alpha_shape_cell_base_3.h>
#include <CGAL/Alpha_shape_vertex_base_3.h>
#include <CGAL/Delaunay_triangulation_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  					ProjTraits;
typedef CGAL::Delaunay_triangulation_2<ProjTraits>			Delaunay2;

typedef CGAL::Alpha_shape_vertex_base_3<K>					Avb3;
typedef CGAL::Alpha_shape_cell_base_3<K>					Acb3;
typedef CGAL::Triangulation_data_structure_3<Avb3, Acb3>	Tds3;
typedef CGAL::Delaunay_triangulation_3<K, Tds3>				Delaunay3;
typedef CGAL::Alpha_shape_3<Delaunay3>						Alpha3;
typedef Alpha3::Alpha_iterator								AlphaIter3;

typedef K::Point_3   										Point3;

typedef CGAL::Point_3<K> 	Point_3;
typedef CGAL::Vector_3<K> 	Vector_3;
typedef CGAL::Ray_3<K> 		Ray_3;
typedef CGAL::Line_3<K> 	Line_3;
typedef CGAL::Segment_3<K> 	Segment_3;

#endif /* INCLUDE_SIM_GEOMETRY_HPP_ */
