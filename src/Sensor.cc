#include "Sensor.hh"
#include "DetectorModule.hh"

void Sensor::check() {
  PropertyObject::check();
  
  if (!numStripsAcross.state() && !pitchEstimate.state()) throw PathfulException("At least one between numStripsAcross and pitchEstimate must be specified");
  if (numStripsAcross.state() && pitchEstimate.state()) throw PathfulException("Only one between numStripsAcross and pitchEstimate can be specified");
  if (!numSegments.state() && !stripLengthEstimate.state()) throw PathfulException("At least one between numSegments and stripLengthEstimate must be specified");
  if (numSegments.state() && stripLengthEstimate.state()) throw PathfulException("Only one between numSegments and stripLengthEstimate can be specified");
}

double Sensor::sensorNormalOffset() const {
  double offset;
  if (parent_->numSensors() <= 1) offset = 0.;
  else {
    if (myid() == 1) offset = -parent_->dsDistance()/2.;
    else offset = parent_->dsDistance()/2.;
  }
  return offset;
}

const Polygon3d<4>& Sensor::hitPoly() const {
  double offset = sensorNormalOffset();
  if (hitPoly_ == 0) hitPoly_ = CoordinateOperations::computeTranslatedPolygon(parent_->basePoly(), offset);
  return *hitPoly_;
}

const Polygon3d<4>& Sensor::hitMidPoly() const {
  if (hitMidPoly_ == 0) hitMidPoly_ = CoordinateOperations::computeMidPolygon(hitPoly());
  return *hitMidPoly_;
}

const Polygon3d<8>& Sensor::envelopePoly() const {
  double envelopeOffset =  sensorThickness() / 2.;
  if (envelopePoly_ == 0) envelopePoly_ = CoordinateOperations::computeEnvelopePolygon<Polygon3d<4>, Polygon3d<8> >(hitPoly(), envelopeOffset); 
  return *envelopePoly_;
}

const Polygon3d<8>& Sensor::envelopeMidPoly() const {
  double envelopeOffset =  sensorThickness() / 2.;
  if (envelopeMidPoly_ == 0) envelopeMidPoly_ = CoordinateOperations::computeEnvelopePolygon<Polygon3d<4>, Polygon3d<8> >(hitMidPoly(), envelopeOffset); 
  return *envelopeMidPoly_;
}

void Sensor::clearPolys() { 
  delete hitPoly_; 
  hitPoly_ = 0;
  delete hitMidPoly_; 
  hitMidPoly_ = 0;
  delete envelopePoly_;
  envelopePoly_ = 0;
  delete envelopeMidPoly_;
  envelopeMidPoly_ = 0;
}

std::pair<XYZVector, int> Sensor::checkHitSegment(const XYZVector& trackOrig, const XYZVector& trackDir) const {
  const Polygon3d<4>& poly = hitPoly();
  XYZVector p;
  if (poly.isLineIntersecting(trackOrig, trackDir, p)) {
    XYZVector v = p - poly.getVertex(0);
    double projL = v.Dot((poly.getVertex(1) - poly.getVertex(0)).Unit());
    return std::make_pair(p, projL / stripLength()); 
  } else return std::make_pair(p, -1);
}



const double Sensor::activeMeanWidth() const { 
  return computeActiveDimension(parent_->meanWidth(), numROCX(), spaceBetweenROCs());
}
const double Sensor::activeMinWidth() const {
  return computeActiveDimension(parent_->minWidth(), numROCX(), spaceBetweenROCs());
}
const double Sensor::activeMaxWidth() const {
  return computeActiveDimension(parent_->maxWidth(), numROCX(), spaceBetweenROCs());
}
const double Sensor::activeLength() const {
  return computeActiveDimension(parent_->length(), numROCY(), spaceBetweenROCs());
}
int Sensor::numStripsAcrossEstimate() const {
  if (numStripsAcross.state()) { return numStripsAcross(); }
  else {
    //std::cout << "activeMeanWidth() = " << activeMeanWidth() << std::endl;
    return round(activeMeanWidth() / pitchEstimate()); }
}
int Sensor::numSegmentsEstimate() const {
  if (numSegments.state()) { return numSegments(); }
  else { 
    //std::cout << "activeLength() = " << activeLength() << std::endl;
    return round(activeLength() / stripLengthEstimate()); }
}
double Sensor::minPitch() const { return activeMinWidth() / numStripsAcrossEstimate(); }
double Sensor::maxPitch() const { return activeMaxWidth() / numStripsAcrossEstimate(); }
double Sensor::pitch() const {
  if (pitchEstimate.state()) { return pitchEstimate(); }
  else { return activeMeanWidth() / numStripsAcrossEstimate(); }
}
double Sensor::stripLength() const { 
  if (stripLengthEstimate.state()) { return stripLengthEstimate(); }
  else { return activeLength() / numSegmentsEstimate(); }
}

double Sensor::alveolaWidth() const { return parent_->meanWidth() / numCrystalsX(); }
double Sensor::alveolaLength() const { return parent_->length() / numCrystalsY(); }


const double Sensor::computeActiveDimension(const double dimension, const int numROCs, const double spaceBetweenROCs) const {
  double activeDimension = dimension;
  if (numROCs >= 2) activeDimension -= (numROCs - 1) * spaceBetweenROCs;
  return activeDimension;
}

define_enum_strings(SensorType) = { "pixel", "largepix", "strip" };
