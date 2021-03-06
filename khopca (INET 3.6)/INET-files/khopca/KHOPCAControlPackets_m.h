//
// Generated file, do not edit! Created by nedtool 5.3 from inet/routing/khopca/KHOPCAControlPackets.msg.
//

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#ifndef __INET_KHOPCACONTROLPACKETS_M_H
#define __INET_KHOPCACONTROLPACKETS_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0503
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif

// dll export symbol
#ifndef INET_API
#  if defined(INET_EXPORT)
#    define INET_API  OPP_DLLEXPORT
#  elif defined(INET_IMPORT)
#    define INET_API  OPP_DLLIMPORT
#  else
#    define INET_API
#  endif
#endif

// cplusplus {{
#include "inet/networklayer/common/L3Address.h"
// }}


namespace inet {

/**
 * Class generated from <tt>inet/routing/khopca/KHOPCAControlPackets.msg:25</tt> by nedtool.
 * <pre>
 * packet KHOPCAWeight
 * {
 *     unsigned int w_n;
 *     L3Address originatorAddr;
 *     L3Address destAddr;
 *     double pow;
 *     double entryTime;
 *     string tree;
 * 
 * }
 * </pre>
 */
class INET_API KHOPCAWeight : public ::omnetpp::cPacket
{
  protected:
    unsigned int w_n;
    L3Address originatorAddr;
    L3Address destAddr;
    double pow;
    double entryTime;
    ::omnetpp::opp_string tree;

  private:
    void copy(const KHOPCAWeight& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const KHOPCAWeight&);

  public:
    KHOPCAWeight(const char *name=nullptr, short kind=0);
    KHOPCAWeight(const KHOPCAWeight& other);
    virtual ~KHOPCAWeight();
    KHOPCAWeight& operator=(const KHOPCAWeight& other);
    virtual KHOPCAWeight *dup() const override {return new KHOPCAWeight(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual unsigned int getW_n() const;
    virtual void setW_n(unsigned int w_n);
    virtual L3Address& getOriginatorAddr();
    virtual const L3Address& getOriginatorAddr() const {return const_cast<KHOPCAWeight*>(this)->getOriginatorAddr();}
    virtual void setOriginatorAddr(const L3Address& originatorAddr);
    virtual L3Address& getDestAddr();
    virtual const L3Address& getDestAddr() const {return const_cast<KHOPCAWeight*>(this)->getDestAddr();}
    virtual void setDestAddr(const L3Address& destAddr);
    virtual double getPow() const;
    virtual void setPow(double pow);
    virtual double getEntryTime() const;
    virtual void setEntryTime(double entryTime);
    virtual const char * getTree() const;
    virtual void setTree(const char * tree);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const KHOPCAWeight& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, KHOPCAWeight& obj) {obj.parsimUnpack(b);}

} // namespace inet

#endif // ifndef __INET_KHOPCACONTROLPACKETS_M_H

