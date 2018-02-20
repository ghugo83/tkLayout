#include "ITCabling/ModulesToELinksConnector.hh"
#include <Tracker.hh>


void ModulesToELinksConnector::visit(Barrel& b) {
  barrelName_ = b.myid();	
}


void ModulesToELinksConnector::visit(Layer& l) {
  layerNumber_ = l.layerNumber();
}
      
     
void ModulesToELinksConnector::visit(BarrelModule& m) {
  const int numELinksPerModule = computeNumELinksPerModule(barrelName_, layerNumber_);

  for (int eLinkIndex = 0; eLinkIndex < numELinksPerModule; eLinkIndex++) {
    // BUILD ELINK IF NECESSARY, AND CONNECT MODULE TO ELINK
    buildELink(m, eLinks_, eLinkIndex);
  }
}


void ModulesToELinksConnector::visit(Endcap& e) {
  endcapName_ = e.myid();
}


void ModulesToELinksConnector::visit(Ring& r)   { 
  ringNumber_ = r.myid();
}


void ModulesToELinksConnector::visit(EndcapModule& m) {
  const int numELinksPerModule = computeNumELinksPerModule(endcapName_, ringNumber_);

  for (int eLinkIndex = 0; eLinkIndex < numELinksPerModule; eLinkIndex++) {
    // BUILD ELINK IF NECESSARY, AND CONNECT MODULE TO ELINK
    buildELink(m, eLinks_, eLinkIndex);
  }
}


/*void ModulesToELinksConnector::postVisit() {
  // CHECK
  checkModulesToELinksCabling(eLinks_);
  }*/


const int ModulesToELinksConnector::computeNumELinksPerModule(const std::string subdetectorName, const int layerOrRingNumber) const {   
  int numELinksPerModule = 0;

  if (subdetectorName == inner_cabling_tbpx) {
    if (layerOrRingNumber == 1) numELinksPerModule = inner_cabling_numELinksPerModuleBarrelLayer1;
    else if (layerOrRingNumber == 2) numELinksPerModule = inner_cabling_numELinksPerModuleBarrelLayer2;
    else if (layerOrRingNumber == 3) numELinksPerModule = inner_cabling_numELinksPerModuleBarrelLayer3;
    else if (layerOrRingNumber == 4) numELinksPerModule = inner_cabling_numELinksPerModuleBarrelLayer4;
    else { 
      logERROR(any2str("Found layer number ") + any2str(layerOrRingNumber)
	       + any2str(" in ") + any2str(inner_cabling_tbpx)
	       + any2str(". This is not supported.")
	       );
    }
  }
  else if (subdetectorName == inner_cabling_tfpx) {
    if (layerOrRingNumber == 1) numELinksPerModule = inner_cabling_numELinksPerModuleForwardRing1;
    else if (layerOrRingNumber == 2) numELinksPerModule = inner_cabling_numELinksPerModuleForwardRing2;
    else if (layerOrRingNumber == 3) numELinksPerModule = inner_cabling_numELinksPerModuleForwardRing3;
    else if (layerOrRingNumber == 4) numELinksPerModule = inner_cabling_numELinksPerModuleForwardRing4;
    else { 
      logERROR(any2str("Found ring number ") + any2str(layerOrRingNumber)
	       + any2str(" in ") + any2str(inner_cabling_tfpx)
	       + any2str(". This is not supported.")
	       );
    }
  }
  else if (subdetectorName == inner_cabling_tepx) {
    numELinksPerModule = inner_cabling_numELinksPerModuleEndcap;
  }
  else { 
    logERROR(any2str("Unknown subdetector ") + any2str(subdetectorName)
	     );
  }

  return numELinksPerModule;
}


/* Build ELink.
 * The eLink is created, and stored in the eLinks_ containers.
 * Lastly, each module is connected to its eLink, and vice-versa.
 */
void ModulesToELinksConnector::buildELink(DetectorModule& m, std::map<std::string, ELink*>& eLinks, const int eLinkIndex) {
  // COMPUTE ELINK ID
  const std::string eLinkId = computeELinkId(m, eLinkIndex);

  // CREATE ELINK IF NECESSARY
  ELink* eLink = nullptr;
  auto found = eLinks.find(eLinkId);
  if (found == eLinks.end()) {
    eLink = createAndStoreELink(eLinks, m, eLinkId);
  }
  else {
    eLink = found->second;
  }

  // CONNECT MODULE TO ELINK
  connectModuleToELink(m, eLink);
}


/* Compute the Id associated to each ELink.
 */
const std::string ModulesToELinksConnector::computeELinkId(DetectorModule& m, const int eLinkIndex) const {
  std::ostringstream eLinkIdStream;
  eLinkIdStream << m.myDetId() << "_" << eLinkIndex;
  std::string eLinkId = eLinkIdStream.str();
  return eLinkId;
}


/*  Create a eLink, if it does not exist yet.
 *  Store it in the eLinks_ container.
 */
ELink* ModulesToELinksConnector::createAndStoreELink(std::map<std::string, ELink*>& eLinks, DetectorModule& m, const std::string eLinkId) {

  //ELink* eLink = GeometryFactory::make<ELink>(m, eLinkId);
  ELink* eLink = GeometryFactory::make<ELink>(eLinkId);

  eLinks.insert(std::make_pair(eLinkId, eLink));
 
  return eLink;
}


/* Connect module to eLink and vice-versa.
 */
void ModulesToELinksConnector::connectModuleToELink(DetectorModule& m, ELink* eLink) const {
  //eLink->addModule(&m);
  m.addELink(eLink);
}

