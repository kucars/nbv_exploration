#include <stdio.h>
#include <math.h>
#include "MeanShift.h"

using namespace std;

#define EPSILON 0.00000001
#define CLUSTER_EPSILON 0.5

double euclidean_distance(const vector<double> &point_a, const vector<double> &point_b){
    double total = 0;
    for(int i=0; i<point_a.size(); i++){
        total += (point_a[i] - point_b[i]) * (point_a[i] - point_b[i]);
    }
    return sqrt(total);
}

double gaussian_kernel(double distance, double kernel_bandwidth){
    double temp =  exp(-1.0/2.0 * (distance*distance) / (kernel_bandwidth*kernel_bandwidth));
    return temp;
}

void MeanShift::set_kernel( double (*_kernel_func)(double,double) ) {
    if(!_kernel_func){
        kernel_func = gaussian_kernel;
    } else {
        kernel_func = _kernel_func;    
    }
}

vector<double> MeanShift::shift_point(const vector<double> &point, const vector<vector<double> > &points, double kernel_bandwidth) {
    vector<double> shifted_point = point;
    for(int dim = 0; dim<shifted_point.size(); dim++){
        shifted_point[dim] = 0;
    }
    double total_weight = 0;
    for(int i=0; i<points.size(); i++){
        vector<double> temp_point = points[i];
        double distance = euclidean_distance(point, temp_point);
        double weight = kernel_func(distance, kernel_bandwidth);
        for(int j=0; j<shifted_point.size(); j++){
            shifted_point[j] += temp_point[j] * weight;
        }
        total_weight += weight;
    }

    for(int i=0; i<shifted_point.size(); i++){
        shifted_point[i] /= total_weight;
    }
    return shifted_point;
}

vector<vector<double> > MeanShift::meanshift(const vector<vector<double> > & points, double kernel_bandwidth, int num_points){
    vector<bool> stop_moving(points.size(), false);

    vector<vector<double> > shifted_points;
    if (points.size() == num_points)
    {
      // Use all points
      shifted_points = points;
    }
    else
    {
      // Copy first few points
      for (int i = 0; i<num_points; i++)
        shifted_points.push_back( points[i] );
    }



    double max_shift_distance;
    do {
        max_shift_distance = 0;
        for(int i=0; i<num_points; i++){
            if (!stop_moving[i]) {
                vector<double>point_new = shift_point(shifted_points[i], points, kernel_bandwidth);
                double shift_distance = euclidean_distance(point_new, shifted_points[i]);
                if(shift_distance > max_shift_distance){
                    max_shift_distance = shift_distance;
                }
                if(shift_distance <= EPSILON) {
                    stop_moving[i] = true;
                }
                shifted_points[i] = point_new;
            }
        }

        printf("\rmax_shift_distance: %f", max_shift_distance);
        fflush(stdout); // Print everything in output buffer
    } while (max_shift_distance > EPSILON);

    printf("\n");

    return shifted_points;
}


vector<vector<double> > MeanShift::meanshift(const vector<vector<double> > & points, double kernel_bandwidth)
{
  return meanshift(points, kernel_bandwidth, points.size() );
}

vector<Cluster> MeanShift::cluster(
    const vector<vector<double> > & points, 
    const vector<vector<double> > & shifted_points,
    int num_points)
{
    vector<Cluster> clusters;

    for (int i = 0; i < num_points; i++) {

        int c = 0;
        for (; c < clusters.size(); c++) {
            if (euclidean_distance(shifted_points[i], clusters[c].mode) <= CLUSTER_EPSILON) {
                break;
            }
        }

        if (c == clusters.size()) {
            Cluster clus;
            clus.mode = shifted_points[i];
            clusters.push_back(clus);
        }

        clusters[c].original_points.push_back(points[i]);
        clusters[c].shifted_points.push_back(shifted_points[i]);
    }

    return clusters;
}


vector<Cluster> MeanShift::cluster(
    const vector<vector<double> > & points,
    const vector<vector<double> > & shifted_points)
{
  return cluster(points, shifted_points, shifted_points.size() );
}


vector<Cluster> MeanShift::run(const vector<vector<double> > & points, double kernel_bandwidth){
    vector<vector<double> > shifted_points = meanshift(points, kernel_bandwidth);
    return cluster(points, shifted_points);
}

vector<Cluster> MeanShift::run(const vector<vector<double> > & points, double kernel_bandwidth, int num_points)
{
  // Use all points if input is -1
  if (num_points == -1)
    num_points = points.size();

  // Use the smaller of the two counts
  if (num_points > points.size())
    num_points = points.size();

  vector<vector<double> > shifted_points = meanshift(points, kernel_bandwidth, num_points);
  return cluster(points, shifted_points);
}
