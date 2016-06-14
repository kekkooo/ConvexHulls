#ifndef COMMONTYPEDEFS_H
#define COMMONTYPEDEFS_H

#include <CGAL/Exact_integer.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_integer.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Box_intersection_d/Box_d.h>
#include <CGAL/Gmpz.h>


typedef unsigned int uint;
//typedef CGAL::Filtered_kernel< CGAL::Cartesian< CGAL::Quotient< CGAL::MP_Float > > > HK;
typedef CGAL::Filtered_kernel< CGAL::Cartesian< CGAL::Epeck::FT > >   HK;
typedef CGAL::Simple_cartesian<double>                                 K;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel         HK;
//typedef CGAL::Exact_predicates_exact_constructions_kernel           HK;
//typedef CGAL::Homogeneous<CGAL::Exact_integer>                      HK;
typedef HK::Point_3                                                 CGAL_Point_3;
typedef CGAL::Polyhedron_3<HK, CGAL::Polyhedron_items_with_id_3>    Polyhedron_3;
typedef CGAL::Polyhedron_3<HK>                                      Polyhedron_3_no_id;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>     Double_Polyhedron_3;
//typedef HK::Segment_3                                               Segment_3;
typedef CGAL::Nef_polyhedron_3<HK>                                  Nef_Polyhedron_3;

#endif // COMMONTYPEDEFS_H
