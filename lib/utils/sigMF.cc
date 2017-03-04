/*
 * sigMF.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: ctriant
 */

#include <phasma/utils/sigMF.h>
#include <sstream>
#include <json/json.h>

sigMF::sigMF (std::string datatype, std::string datapath, std::string version,
	      size_t capture_sample_start, size_t annotation_sample_start,
	      size_t sample_count)
{
  d_datatype = datatype;
  d_datapath = datapath;
  d_version = version;
  d_capture_sample_start = capture_sample_start;
  d_annotation_samples_start = annotation_sample_start;
  d_sample_count = sample_count;
  d_freq_upper_edge = 0;
  d_freq_lower_edge = 0;
  d_offset = 0;
  d_sample_rate = 0;
  d_frequency = 0;
}

sigMF::sigMF (std::string datatype, std::string datapath, std::string version,
	      size_t capture_sample_start, double sample_rate, std::string time,
	      size_t annotation_sample_start, size_t sample_count,
	      double freq_lower_edge, double freq_upper_edge,
	      std::string comment)
{
  d_datatype = datatype;
  d_datapath = datapath;
  d_version = version;
  d_capture_sample_start = capture_sample_start;
  d_sample_rate = sample_rate;
  d_time = time;
  d_annotation_samples_start = annotation_sample_start;
  d_sample_count = sample_count;
  d_freq_upper_edge = freq_upper_edge;
  d_freq_lower_edge = freq_lower_edge;
  d_comment = comment;
  d_offset = 0;
  d_frequency = 0;
}

size_t
sigMF::getAnnotationSamplesStart () const
{
  return d_annotation_samples_start;
}

void
sigMF::setAnnotationSamplesStart (size_t annotationSamplesStart)
{
  d_annotation_samples_start = annotationSamplesStart;
}

const std::string&
sigMF::getAuthor () const
{
  return d_author;
}

void
sigMF::setAuthor (const std::string& author)
{
  d_author = author;
}

size_t
sigMF::getCaptureSampleStart () const
{
  return d_capture_sample_start;
}

void
sigMF::setCaptureSampleStart (size_t captureSampleStart)
{
  d_capture_sample_start = captureSampleStart;
}

const std::string&
sigMF::getComment () const
{
  return d_comment;
}

void
sigMF::setComment (const std::string& comment)
{
  d_comment = comment;
}

const std::string&
sigMF::getDatapath () const
{
  return d_datapath;
}

void
sigMF::setDatapath (const std::string& datapath)
{
  d_datapath = datapath;
}

const std::string&
sigMF::getDatatype () const
{
  return d_datatype;
}

void
sigMF::setDatatype (const std::string& datatype)
{
  d_datatype = datatype;
}

const std::string&
sigMF::getDate () const
{
  return d_date;
}

void
sigMF::setDate (const std::string& date)
{
  d_date = date;
}

const std::string&
sigMF::getDescription () const
{
  return d_description;
}

void
sigMF::setDescription (const std::string& description)
{
  d_description = description;
}

double
sigMF::getFreqLowerEdge () const
{
  return d_freq_lower_edge;
}

void
sigMF::setFreqLowerEdge (double freqLowerEdge)
{
  d_freq_lower_edge = freqLowerEdge;
}

double
sigMF::getFreqUpperEdge () const
{
  return d_freq_upper_edge;
}

void
sigMF::setFreqUpperEdge (double freqUpperEdge)
{
  d_freq_upper_edge = freqUpperEdge;
}

double
sigMF::getFrequency () const
{
  return d_frequency;
}

void
sigMF::setFrequency (double frequency)
{
  d_frequency = frequency;
}

const std::string&
sigMF::getHw () const
{
  return d_hw;
}

void
sigMF::setHw (const std::string& hw)
{
  d_hw = hw;
}

const std::string&
sigMF::getLicense () const
{
  return d_license;
}

void
sigMF::setLicense (const std::string& license)
{
  d_license = license;
}

uint64_t
sigMF::getOffset () const
{
  return d_offset;
}

void
sigMF::setOffset (uint64_t offset)
{
  d_offset = offset;
}

size_t
sigMF::getSampleCount () const
{
  return d_sample_count;
}

void
sigMF::setSampleCount (size_t sampleCount)
{
  d_sample_count = sampleCount;
}

double
sigMF::getSampleRate () const
{
  return d_sample_rate;
}

void
sigMF::setSampleRate (double sampleRate)
{
  d_sample_rate = sampleRate;
}

const std::string&
sigMF::getSha512 () const
{
  return d_sha512;
}

void
sigMF::setSha512 (const std::string& sha512)
{
  d_sha512 = sha512;
}

const std::string&
sigMF::getTime () const
{
  return d_time;
}

void
sigMF::setTime (const std::string& time)
{
  d_time = time;
}

const std::string&
sigMF::getVersion () const
{
  return d_version;
}

void
sigMF::setVersion (const std::string& version)
{
  d_version = version;
}

std::string
sigMF::toJSON ()
{
  // TODO: Support all fields
  Json::Value root;
  Json::Value global;
  Json::Value capture;
  Json::Value capture_list (Json::arrayValue);
  Json::Value annotation;

  global["core:datapath"] = d_datapath;
  global["core:datatype"] = d_datatype;

  capture["core:sample_start"] = d_capture_sample_start;
  capture["core:sample_rate"] = d_sample_rate;
  capture["core:time"] = d_time;

  capture_list.append (capture);

  annotation["core:sample_start"] = d_annotation_samples_start;
  annotation["core:sample_count"] = d_sample_count;
  annotation["core:freq_lower_edge"] = d_freq_lower_edge;
  annotation["core:freq_upper_edge"] = d_freq_upper_edge;
  annotation["core:comment"] = d_comment;
  annotation["span"] = d_freq_upper_edge - d_freq_lower_edge + 20e3;

  root["global"] = global;
  root["capture"] = capture_list;
  root["annotation"] = annotation;

  std::string json = root.toStyledString ();

  return json;
}

sigMF::~sigMF ()
{
}

