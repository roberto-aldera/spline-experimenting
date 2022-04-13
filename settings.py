DATA_ROOT_PATH = "/Users/roberto/data/"
# RO_PATH = DATA_ROOT_PATH + "RadarDataLogs/2017-08-18-11-21-04-oxford-10k-with-radar-1/" \
#                            "logs/radar/cts350x/2017-08-18-10-21-06/ro_relative_poses.monolithic"
# INS_PATH = DATA_ROOT_PATH + "RadarDataLogs/2017-08-18-11-21-04-oxford-10k-with-radar-1/" \
#                             "logs/INS/NovAtel/2017-08-18-10-21-20/flattened_novatel_generated_poses.monolithic"

RO_PATH = DATA_ROOT_PATH + "RadarDataLogs/2020-01-31-14-46-47-medium-sax-rainy-bumpy/radar/" \
                           "cts350x_102717/2020-01-31-14-46-54/ro_relative_poses.monolithic"
INS_PATH = DATA_ROOT_PATH + "RadarDataLogs/2020-01-31-14-46-47-medium-sax-rainy-bumpy/INS/relative_poses.monolithic"
OUTPUT_ROOT_PATH = DATA_ROOT_PATH + "splines-python/medium-sax/"
FIGURE_PATH = OUTPUT_ROOT_PATH
RO_CSV = OUTPUT_ROOT_PATH + "ro_poses.csv"
RO_ALIGNED_CSV = OUTPUT_ROOT_PATH + "ro_aligned_poses.csv"
SPLINE_CSV = OUTPUT_ROOT_PATH + "spline_poses.csv"
SPLINE_ALIGNED_CSV = OUTPUT_ROOT_PATH + "spline_aligned_poses.csv"
INTERPOLATED_INS_CSV = OUTPUT_ROOT_PATH + "interpolated_ins_poses.csv"
INS_FILTERED_INTERPOLATED_CSV = OUTPUT_ROOT_PATH + "filtered_interpolated_ins_poses.csv"
