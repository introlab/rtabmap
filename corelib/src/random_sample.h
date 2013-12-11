/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: extract_indices.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#ifndef PCL_FILTERS_RANDOM_SUBSAMPLE_H_
#define PCL_FILTERS_RANDOM_SUBSAMPLE_H_

#include <pcl/filters/filter_indices.h>
#include <time.h>
#include <limits.h>

  /** \brief @b RandomSample applies a random sampling with uniform probability.
    * Based off Algorithm A from the paper "Faster Methods for Random Sampling"
    * by Jeffrey Scott Vitter. The algorithm runs in O(N) and results in sorted
    * indices
    * http://www.ittc.ku.edu/~jsv/Papers/Vit84.sampling.pdf
    * \author Justin Rosen
    * \ingroup filters
    */
  template<typename PointT>
  class RandomSample : public pcl::FilterIndices<PointT>
  {
    using pcl::FilterIndices<PointT>::filter_name_;
    using pcl::FilterIndices<PointT>::getClassName;
    using pcl::FilterIndices<PointT>::indices_;
    using pcl::FilterIndices<PointT>::input_;
    using pcl::FilterIndices<PointT>::negative_;
    using pcl::FilterIndices<PointT>::keep_organized_;
    using pcl::FilterIndices<PointT>::user_filter_value_;
    using pcl::FilterIndices<PointT>::extract_removed_indices_;
    using pcl::FilterIndices<PointT>::removed_indices_;

    typedef typename pcl::FilterIndices<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:

      typedef boost::shared_ptr< RandomSample<PointT> > Ptr;
      typedef boost::shared_ptr< const RandomSample<PointT> > ConstPtr;

      /** \brief Empty constructor. */
      RandomSample (bool extract_removed_indices = false) :
    	  pcl::FilterIndices<PointT> (extract_removed_indices),
        sample_ (UINT_MAX),
        seed_ (static_cast<unsigned int> (time (NULL)))
      {
        filter_name_ = "RandomSample";
      }

      /** \brief Set number of indices to be sampled.
        * \param sample
        */
      inline void
      setSample (unsigned int sample)
      {
        sample_ = sample;
      }

      /** \brief Get the value of the internal \a sample parameter.
        */
      inline unsigned int
      getSample ()
      {
        return (sample_);
      }

      /** \brief Set seed of random function.
        * \param seed
        */
      inline void
      setSeed (unsigned int seed)
      {
        seed_ = seed;
      }

      /** \brief Get the value of the internal \a seed parameter.
        */
      inline unsigned int
      getSeed ()
      {
        return (seed_);
      }

    protected:

      /** \brief Number of indices that will be returned. */
      unsigned int sample_;
      /** \brief Random number seed. */
      unsigned int seed_;

      /** \brief Sample of point indices into a separate PointCloud
        * \param output the resultant point cloud
        */
      void
      applyFilter (PointCloud &output)
      {
        std::vector<int> indices;
        if (keep_organized_)
        {
          bool temp = extract_removed_indices_;
          extract_removed_indices_ = true;
          applyFilter (indices);
          extract_removed_indices_ = temp;
          copyPointCloud (*input_, output);
          // Get X, Y, Z fields
          std::vector<sensor_msgs::PointField> fields;
          pcl::getFields (*input_, fields);
          std::vector<size_t> offsets;
          for (size_t i = 0; i < fields.size (); ++i)
          {
            if (fields[i].name == "x" ||
                fields[i].name == "y" ||
                fields[i].name == "z")
              offsets.push_back (fields[i].offset);
          }
          // For every "removed" point, set the x,y,z fields to user_filter_value_
          const static float user_filter_value = user_filter_value_;
          for (size_t rii = 0; rii < removed_indices_->size (); ++rii)
          {
            uint8_t* pt_data = reinterpret_cast<uint8_t*> (&output[(*removed_indices_)[rii]]);
            for (size_t i = 0; i < offsets.size (); ++i)
            {
              memcpy (pt_data + offsets[i], &user_filter_value, sizeof (float));
            }
            if (!pcl_isfinite (user_filter_value_))
              output.is_dense = false;
          }
        }
        else
        {
          output.is_dense = true;
          applyFilter (indices);
          copyPointCloud (*input_, indices, output);
        }
      }

      /** \brief Sample of point indices
        * \param indices the resultant point cloud indices
        */
      void
      applyFilter (std::vector<int> &indices)
      {
        unsigned N = static_cast<unsigned> (indices_->size ());

        unsigned int sample_size = negative_ ? N - sample_ : sample_;
        // If sample size is 0 or if the sample size is greater then input cloud size
        //   then return all indices
        if (sample_size >= N)
        {
          indices = *indices_;
          removed_indices_->clear ();
        }
        else
        {
          // Resize output indices to sample size
          indices.resize (static_cast<size_t> (sample_size));
          if (extract_removed_indices_)
            removed_indices_->resize (static_cast<size_t> (N - sample_size));

          // Set random seed so derived indices are the same each time the filter runs
          std::srand (seed_);

          // Algorithm A
          unsigned top = N - sample_size;
          unsigned i = 0;
          unsigned index = 0;
          std::vector<bool> added;
          if (extract_removed_indices_)
            added.resize (indices_->size (), false);
          for (size_t n = sample_size; n >= 2; n--)
          {
            float V = unifRand ();
            unsigned S = 0;
            float quot = static_cast<float> (top) / static_cast<float> (N);
            while (quot > V)
            {
              S++;
              top--;
              N--;
              quot = quot * static_cast<float> (top) / static_cast<float> (N);
            }
            index += S;
            if (extract_removed_indices_)
              added[index] = true;
            indices[i++] = (*indices_)[index++];
            N--;
          }

          index += N * static_cast<unsigned> (unifRand ());
          if (extract_removed_indices_)
            added[index] = true;
          indices[i++] = (*indices_)[index++];

          // Now populate removed_indices_ appropriately
          if (extract_removed_indices_)
          {
            unsigned ri = 0;
            for (size_t i = 0; i < added.size (); i++)
            {
              if (!added[i])
              {
                (*removed_indices_)[ri++] = (*indices_)[i];
              }
            }
          }
        }
      }

      /** \brief Return a random number fast using a LCG (Linear Congruential Generator) algorithm.
        * See http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/ for more information.
        */
      inline float
      unifRand ()
      {
        return (static_cast<float>(rand () / double (RAND_MAX)));
        //return (((214013 * seed_ + 2531011) >> 16) & 0x7FFF);
      }
  };


#endif  //#ifndef PCL_FILTERS_RANDOM_SUBSAMPLE_H_
