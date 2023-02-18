

#include <vector>
#include <iostream>
#include "laszip/laszip_api.h"
#include <Eigen/Eigen>

void adjustHeader(laszip_header* output_header, laszip_header* input_header, const Eigen::Affine3d & mat )
{
    *output_header = *input_header;
    const Eigen::Vector3d max{output_header->max_x,output_header->max_y,output_header->max_z};
    const Eigen::Vector3d min{output_header->min_x,output_header->min_y,output_header->min_z};
    const Eigen::Vector3d offset{output_header->x_offset,output_header->y_offset,output_header->z_offset};

    const Eigen::Vector3d adj_max = mat*max;
    const Eigen::Vector3d adj_min = mat*min;
    const Eigen::Vector3d adj_off = mat*offset;

    output_header->max_x=adj_max.x();
    output_header->max_y=adj_max.y();
    output_header->max_z=adj_max.z();

    output_header->min_x=adj_min.x();
    output_header->min_y=adj_min.y();
    output_header->min_z=adj_min.z();

    output_header->x_offset=adj_off.x();
    output_header->y_offset=adj_off.y();
    output_header->z_offset=adj_off.z();
}

void adjustPoint(laszip_F64 output_coordinates[3],laszip_F64 input_coordinates[3], const Eigen::Affine3d & mat )
{
    Eigen::Vector3d i{input_coordinates[0],input_coordinates[1],input_coordinates[2]};
    Eigen::Vector3d o = mat * i;
    output_coordinates[0] = o.x();
    output_coordinates[1] = o.y();
    output_coordinates[2] = o.z();
}

int main(int argc, char* argv[])
{



    if (argc!=3+6){
        std::cerr << "need 2 argumenst :\n";
        std::cerr << argv[0] << " input.laz output.laz";
        std::abort();
    }
    Eigen::Matrix4d adjusment{Eigen::Matrix4d::Identity()};
    adjusment << atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),
                 atof(argv[7]),atof(argv[8]),atof(argv[9]),atof(argv[10]),
                 atof(argv[11]),atof(argv[12]),atof(argv[13]),atof(argv[14]);
    Eigen::Affine3d transform {adjusment};

    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr,"DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_POINTER laszip_writer;
    if (laszip_create(&laszip_writer))
    {
      fprintf(stderr,"DLL ERROR: creating laszip reader\n");
      std::abort();
    }

    const std::string file_name_in =argv[1];
    const std::string file_name_out =argv[2];

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, file_name_in.c_str(), &is_compressed))
    {
        fprintf(stderr,"DLL ERROR: opening laszip reader for '%s'\n", file_name_in.c_str());
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;


    laszip_header* header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr,"DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }

    adjustHeader(header,header, transform);

    if (laszip_set_header(laszip_writer, header))
    {
      fprintf(stderr,"DLL ERROR: setting header pointer from laszip reader\n");
      std::abort();
    }

    fprintf(stderr,"file '%s' contains %u points\n", file_name_in.c_str(), header->number_of_point_records);

    if (laszip_open_writer(laszip_writer, file_name_out.c_str(), is_compressed))
    {
      fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", file_name_out.c_str());
      return false;
    }

    laszip_point* input_point;
    if (laszip_get_point_pointer(laszip_reader, &input_point))
    {
        fprintf(stderr,"DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    laszip_point* output_point;
    if (laszip_get_point_pointer(laszip_writer, &output_point))
    {
      fprintf(stderr,"DLL ERROR: getting point pointer from laszip reader\n");
      std::abort();
    }


    for (int i =0; i <  header->number_of_point_records; i++)
    {
      if (laszip_read_point(laszip_reader))
      {
        fprintf(stderr,"DLL ERROR: reading point %u\n", i);
        std::abort();
      }

      laszip_F64 input_coordinates[3];
      if (laszip_get_coordinates(laszip_reader, input_coordinates))
      {
        fprintf(stderr,"DLL ERROR: laszip_set_coordinates %u\n", i);
        std::abort();
      }
      *output_point = *input_point;

      laszip_F64 output_coordinates[3];
      adjustPoint(output_coordinates,input_coordinates,transform);

      if (laszip_set_coordinates(laszip_writer, output_coordinates))
      {
        fprintf(stderr,"DLL ERROR: laszip_set_coordinates %u\n", i);
        std::abort();
      }

      if (laszip_write_point(laszip_writer)) {
        fprintf(stderr, "DLL ERROR: writing point %I64d\n", i);
        return false;
      }
    }

    // close the reader

    if (laszip_close_reader(laszip_reader))
    {
      fprintf(stderr, "DLL ERROR: closing laszip reader\n");
      return false;
    }

    // destroy the reader

    if (laszip_destroy(laszip_reader))
    {
      fprintf(stderr, "DLL ERROR: destroying laszip reader\n");
      return false;
    }

    laszip_I64 p_count{0};
    if (laszip_get_point_count(laszip_writer, &p_count))
    {
      fprintf(stderr, "DLL ERROR: getting point count\n");
      return false;
    }

    fprintf(stderr, "successfully written %ld points\n", p_count);

    // close the writer

    if (laszip_close_writer(laszip_writer))
    {
      fprintf(stderr, "DLL ERROR: closing laszip writer\n");
      return false;
    }

    // destroy the writer

    if (laszip_destroy(laszip_writer))
    {
      fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
      return false;
    }

}