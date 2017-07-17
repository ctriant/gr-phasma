/*
 * sigMF.cpp
 *
 *  Created on: Feb 24, 2017
 *      Author: ctriant
 */

#include <phasma/utils/sigMF.h>
#include <sstream>
#include <fstream>
#include <stdio.h>

sigMF::sigMF (std::string datatype, std::string datapath, std::string version)
{
  d_datatype = datatype;
  d_datapath = datapath;
  d_version = version;
  d_capture_sample_start = 0;
  d_annotation_samples_start = 0;
  d_sample_count = 0;
  d_freq_upper_edge = 0;
  d_freq_lower_edge = 0;
  d_offset = 0;
  d_sample_rate = 0;
  d_frequency = 0;
  
  this->add_global();
  
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
  d_root["annotations"][0]["core::comment"] = comment;
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

/* TODO: Modify parameters */
void
sigMF::add_global ()
{
  d_global["core:datapath"] = d_datapath;
  d_global["core:datatype"] = d_datatype;
  d_root["global"] = d_global;
}

void
sigMF::add_capture (size_t sample_start, double sample_rate)
{
  Json::Value capture;
  
  d_capture_list = d_root["capture"];

  capture["core:sample_start"] = sample_start;
  capture["core:sample_rate"] = sample_rate;

  d_capture_list.append (capture);
  d_root["capture"] = d_capture_list;
}

void
sigMF::add_capture (Json::Value capture)
{
  d_root["capture"].append (capture);
}

void
sigMF::add_annotation (size_t samples_start, size_t sample_count,
		       std::string comment, double freq_lower_edge,
		       double freq_upper_edge, float decimation, 
		       std::string time)
{
  Json::Value annotation;

  d_annotation_list = d_root["annotations"];

  annotation["core:sample_start"] = samples_start;
  annotation["core:sample_count"] = sample_count;
  annotation["core:freq_lower_edge"] = freq_lower_edge;
  annotation["core:freq_upper_edge"] = freq_upper_edge;
  annotation["core:decimation"] = decimation;
  annotation["core:time"] = time;

  d_annotation_list.append (annotation);
  d_root["annotations"] = d_annotation_list;
}

void
sigMF::add_annotation (Json::Value annotation)
{
  d_root["annotations"].append (annotation);
}

void
sigMF::setRoot (Json::Value root)
{
  d_root = root;
}

Json::Value
sigMF::getRoot ()
{
  return d_root;
}

Json::Value
sigMF::get_annotation() {
  return d_root["annotations"];
}

Json::Value
sigMF::get_capture() {
  return d_root["capture"];
}

void
sigMF::parse_file (std::string json_sigmf)
{
  Json::Reader reader;
  Json::Value tmp;
  std::ifstream file(json_sigmf);
  
  bool success = reader.parse (file, tmp, false);
  if (success) {
    d_root = tmp;
  }
  
}

void
sigMF::parse_string (std::string json_sigmf, std::string decision)
{
  Json::Reader reader;
  Json::Value tmp;
  
  bool success = reader.parse (json_sigmf, tmp, false);
  if (success) {
    d_root = tmp;
    d_root["annotations"][0]["core::comment"] = decision;
  }
  
}

std::string
sigMF::toJSON ()
{
  // TODO: Support all fields
  std::string json = d_root.toStyledString ();

  return json;
}

sigMF::~sigMF ()
{
}

