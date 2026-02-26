#include "rtls_adapters.h"

namespace RTLSAdapters
{

isas_msgs::msg::RTLSStick fraunhoferToIsas(const fraunhofer_rtls_flare::msg::RTLSStick& uwb_msg)
{
    isas_msgs::msg::RTLSStick converted;
    converted.header = uwb_msg.header;
    converted.id = uwb_msg.id;
    converted.t = uwb_msg.t;
    converted.pos = uwb_msg.pos;
    converted.mean = uwb_msg.mean;
    converted.hrp = uwb_msg.hrp;
    converted.noga = uwb_msg.noga;
    converted.nora = uwb_msg.nora;
    converted.ranges.reserve(uwb_msg.ranges.size());
    for (const auto& rg : uwb_msg.ranges) {
        if (rg.ra == 0) continue;
        isas_msgs::msg::RTLSRange rg_c;
        rg_c.id = rg.id;
        rg_c.pos = rg.pos;
        rg_c.pr = rg.pr;
        rg_c.range = rg.range;
        rg_c.mean = rg.mean;
        rg_c.var = rg.var;
        rg_c.fpp = rg.fpp;
        rg_c.rxp = rg.rxp;
        rg_c.csn = rg.csn;
        rg_c.cmn = rg.cmn;
        rg_c.toc = rg.toc;
        rg_c.ra = rg.ra;
        converted.ranges.push_back(rg_c);
    }
    return converted;
}

isas_msgs::msg::Anchorlist fraunhoferToIsas(const fraunhofer_rtls_flare::msg::Anchorlist& anchor_msg)
{
    isas_msgs::msg::Anchorlist converted;
    converted.anchor.reserve(anchor_msg.anchor.size());
    for (const auto& anchor : anchor_msg.anchor) {
        isas_msgs::msg::AnchorPosition anchor_c;
        anchor_c.id = anchor.id;
        anchor_c.position = anchor.position;
        converted.anchor.push_back(anchor_c);
    }
    return converted;
}

} // namespace RTLSAdapters
