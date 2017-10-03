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

// TO DO: USE DIRECTLY A POLYGON OF TEMPLATED SIZE AND NOT A VECTOR OF XYZVECTOR IN GEOMETRIC MODULE.
// + Handle Inner Tracker case
Polygon3d<10>& Sensor::hybridsPoly() const {
  
  if (hybridsPoly_ == 0) {
    const int contourSize = parent_->contour().size();
    if (contourSize != 0) {
    
      // Our local axes in global coordinates
      XYZVector ex, ey;
      ey = hitPoly().getVertex(0) - hitPoly().getVertex(1) ;
      ex = hitPoly().getVertex(2) - hitPoly().getVertex(1) ;
      XYZVector center = hitPoly().getCenter();
      ex = ex / sqrt(ex.Mag2());
      ey = ey / sqrt(ey.Mag2());

      //Polygon3d<10> poly;
      for (int i = 0; i < contourSize; i++) {
	const XYZVector& contourLocal = parent_->contour().at(i);
	XYZVector contourGlobal = ex * contourLocal.X() + ey * contourLocal.Y() + center;
	*hybridsPoly_ << contourGlobal;
      }
      //*hybridsPoly_ = poly;

      //std::cout << "contourGlobal.X() = " << contourGlobal.X() << "contourGlobal.Y() = " << contourGlobal.Y() << "contourGlobal.Z() = " << contourGlobal.Z() << std::endl;
    }
    // Do not care about Inner Tracker case here, since private branch for Outer Tracker!
    else {
      //Polygon3d<10> fakePoly;
      for (int i = 0; i < 10; i++) {
	*hybridsPoly_ << XYZVector( 1., 1., 1.);
      }
      //hybridsPoly_ = &fakePoly;
    }

  }

  return *hybridsPoly_;
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
  delete hybridsPoly_; 
  hybridsPoly_ = 0;
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

int Sensor::numStripsAcrossEstimate() const {
  if (numStripsAcross.state()) return numStripsAcross();
  else return floor(parent_->meanWidth() / pitchEstimate() + 0.5);
}
int Sensor::numSegmentsEstimate() const {
  if (numSegments.state()) return numSegments();
  else return floor(parent_->length() / stripLengthEstimate() + 0.5);
}
double Sensor::minPitch() const { return parent_->minWidth() / (double)numStripsAcrossEstimate(); }
double Sensor::maxPitch() const { return parent_->maxWidth() / (double)numStripsAcrossEstimate(); }
double Sensor::pitch() const { return parent_->meanWidth() / (double)numStripsAcrossEstimate(); }
double Sensor::stripLength() const { return parent_->length() / numSegmentsEstimate(); }

double Sensor::alveolaWidth() const { return parent_->meanWidth() / (double)numCrystalsX(); }
double Sensor::alveolaLength() const { return parent_->length() / (double)numCrystalsY(); }

define_enum_strings(SensorType) = { "pixel", "largepix", "strip" };
