/*
 * sigMF.h
 *
 *  Created on: Feb 24, 2017
 *      Author: ctriant
 */

#ifndef LIB_UTILS_SIGMF_H_
#define LIB_UTILS_SIGMF_H_

#include <string>

class sigMF
{
private:

  /* Global object fields */
  std::string d_datatype;
  std::string d_datapath;
  std::string d_version;
  std::string d_sha512;
  uint64_t d_offset;
  std::string d_description;
  std::string d_author;
  std::string d_date;
  std::string d_license;
  std::string d_hw;

  /* Capture object fileds */
  size_t d_capture_sample_start;
  double d_frequency;
  double d_sample_rate;
  std::string d_time;

  /* Annotation object fileds */
  size_t d_annotation_samples_start;
  size_t d_sample_count;
  std::string d_comment;
  double d_freq_lower_edge;
  double d_freq_upper_edge;

public:
  sigMF (std::string datatype, std::string datapath, std::string version,
	 size_t capture_sample_start, size_t annotation_sample_start,
	 size_t sample_count);

  sigMF (std::string datatype, std::string datapath, std::string version,
  	 size_t capture_sample_start, double sample_rate, std::string time,
	 size_t annotation_sample_start, size_t sample_count, double freq_lower_edge,
	 double freq_upper_edge, std::string comment);

  virtual
  ~sigMF ();
  size_t
  getAnnotationSamplesStart () const;
  void
  setAnnotationSamplesStart (size_t annotationSamplesStart);
  const std::string&
  getAuthor () const;
  void
  setAuthor (const std::string& author);
  size_t
  getCaptureSampleStart () const;
  void
  setCaptureSampleStart (size_t captureSampleStart);
  const std::string&
  getComment () const;
  void
  setComment (const std::string& comment);
  const std::string&
  getDatapath () const;
  void
  setDatapath (const std::string& datapath);
  const std::string&
  getDatatype () const;
  void
  setDatatype (const std::string& datatype);
  const std::string&
  getDate () const;
  void
  setDate (const std::string& date);
  const std::string&
  getDescription () const;
  void
  setDescription (const std::string& description);
  double
  getFreqLowerEdge () const;
  void
  setFreqLowerEdge (double freqLowerEdge);
  double
  getFreqUpperEdge () const;
  void
  setFreqUpperEdge (double freqUpperEdge);
  double
  getFrequency () const;
  void
  setFrequency (double frequency);
  const std::string&
  getHw () const;
  void
  setHw (const std::string& hw);
  const std::string&
  getLicense () const;
  void
  setLicense (const std::string& license);
  uint64_t
  getOffset () const;
  void
  setOffset (uint64_t offset);
  size_t
  getSampleCount () const;
  void
  setSampleCount (size_t sampleCount);
  double
  getSampleRate () const;
  void
  setSampleRate (double sampleRate);
  const std::string&
  getSha512 () const;
  void
  setSha512 (const std::string& sha512);
  const std::string&
  getTime () const;
  void
  setTime (const std::string& time);
  const std::string&
  getVersion () const;
  void
  setVersion (const std::string& version);
  std::string
  toJSON();
};

#endif /* LIB_UTILS_SIGMF_H_ */
