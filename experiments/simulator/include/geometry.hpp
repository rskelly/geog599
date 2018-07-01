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

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Projection_traits_xy_3<K>  					Gt;
typedef CGAL::Delaunay_triangulation_2<Gt> 					Delaunay;
typedef K::Point_3   										Point;

typedef CGAL::Point_3<K> 	Point_3;
typedef CGAL::Vector_3<K> 	Vector_3;
typedef CGAL::Ray_3<K> 		Ray_3;
typedef CGAL::Line_3<K> 	Line_3;
typedef CGAL::Segment_3<K> 	Segment_3;

#endif /* INCLUDE_SIM_GEOMETRY_HPP_ */
