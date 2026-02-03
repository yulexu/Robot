// # How to modify ChannelIndex so that we can change frequency/wavelength
// # 1.Find the rfid.cpp in ~/src/rfid/src
// # 2.Find codes below and set HopTableID ,ChannelIndex and TransmitPower
//         # // modified CRFTransimitter
//         # CRFTransmitter        *pinlv = new(CRFTransmitter);
//         # pinlv->setHopTableID(21);
//         # pinlv->setChannelIndex(4);
//         # pinlv->setTransmitPower(81);
//         # pAnt->setRFTransmitter(pinlv);
// # 3.catkin_make for your workspace
// # 4.rosrun rfid rfid 169.254.1.1(roscore or roslaunch before rosrun)

src/rfid_v1.cpp modified code not match windows config, fast read speed;
src/rfid_v2.cpp modified veersion 2, match windows, slow read speed
