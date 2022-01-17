/**
 * SORT: A Simple, Online and Realtime Tracker
 *
 * This is a C++ implementation of the 2016 by Alex Bewley proposed tracking
 * algorithm. See https://github.com/abewley/sort and
 * @inproceedings{sort2016,
 * author = {Bewley, Alex and Ge, Zongyuan and Ott, Lionel and Ramos, Fabio and Upcroft, Ben},
 * booktitle = {2016 IEEE International Conference on Image Processing (ICIP)},
 * title = {Simple online and realtime tracking},
 * year = {2016},
 * pages = {3464-3468}
 * }
 * for further information.
 * This was written by Falko Becker, tyler.newnoise@gmail.com, 2020.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SORT_SORT_H
#define SORT_SORT_H
#ifndef NDEBUG
#define NDEBUG
#endif
#include <algorithm>
#include <map>
#include <unordered_map>
#include <vector>

#include <dlib/optimization/max_cost_assignment.h>
#include <opencv2/video/tracking.hpp>

#if !defined(NDEBUG)
#include <string>
#endif

namespace sort {

typedef cv::Rect_<float> BBox;

struct Track {
  BBox bbox;
  std::size_t id{};
};
/**
 * This class implements a simple multi target tracker. It holds an
 * unordered map with the track ids as the key and a KalmanBoxTracker object as
 * the value.
 */
class SORT {
public:
  SORT(const double iou_threshold, const unsigned int max_age,
          const unsigned int n_init);

  ~SORT() = default;

  [[nodiscard]] std::vector<struct Track> update(const std::vector<BBox >& detections);

private:
  /**
   * This nested class implements a single target track with state
   * [x, y, s, r, x', y', s'], where x,y are the center, s is the scale/area
   * and r the aspect ratio of the bounding box, and x', y', s' their
   * respective velocities.
   */
  class KalmanBoxTracker {
  public:
    explicit KalmanBoxTracker(const BBox& bbox);

    ~KalmanBoxTracker() = default;

    [[nodiscard]] BBox get_state() const;

    void predict();

    void update(const BBox& bbox);

    std::size_t hits;
    std::size_t time_since_update;

  private:
    cv::KalmanFilter kf_ = cv::KalmanFilter(7, 4, 0);

    static cv::Mat bbox_to_z(const BBox& bbox);

    static BBox x_to_bbox(const cv::Mat& state);
  }; // class KalmanBoxTracker

  struct Match_{
    std::size_t detection_id{};
    std::size_t track_id{};
  };

  void associate_detections_to_trackers_(
          const std::vector<BBox>&,
          std::vector<struct Match_>&,
          std::vector<std::size_t>&);

  static float iou_(const BBox&, const BBox&);

  std::size_t id_;
  std::size_t frame_cnt_;
  unsigned int max_age_;
  unsigned int min_hits_;
  const double precision_;
  std::unordered_map<std::size_t, std::unique_ptr<KalmanBoxTracker>> trackers_;
  double iou_threshold_;
};

} // namespace sort
#endif
