//
// Generated file, do not edit! Created by nedtool 5.3 from inet/routing/khopca/KHOPCAControlPackets.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wshadow"
#  pragma clang diagnostic ignored "-Wconversion"
#  pragma clang diagnostic ignored "-Wunused-parameter"
#  pragma clang diagnostic ignored "-Wc++98-compat"
#  pragma clang diagnostic ignored "-Wunreachable-code-break"
#  pragma clang diagnostic ignored "-Wold-style-cast"
#elif defined(__GNUC__)
#  pragma GCC diagnostic ignored "-Wshadow"
#  pragma GCC diagnostic ignored "-Wconversion"
#  pragma GCC diagnostic ignored "-Wunused-parameter"
#  pragma GCC diagnostic ignored "-Wold-style-cast"
#  pragma GCC diagnostic ignored "-Wsuggest-attribute=noreturn"
#  pragma GCC diagnostic ignored "-Wfloat-conversion"
#endif

#include <iostream>
#include <sstream>
#include "KHOPCAControlPackets_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: No doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp

namespace inet {

// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

Register_Class(KHOPCAWeight)

KHOPCAWeight::KHOPCAWeight(const char *name, short kind) : ::omnetpp::cPacket(name,kind)
{
    this->w_n = 0;
    this->pow = 0;
    this->entryTime = 0;
}

KHOPCAWeight::KHOPCAWeight(const KHOPCAWeight& other) : ::omnetpp::cPacket(other)
{
    copy(other);
}

KHOPCAWeight::~KHOPCAWeight()
{
}

KHOPCAWeight& KHOPCAWeight::operator=(const KHOPCAWeight& other)
{
    if (this==&other) return *this;
    ::omnetpp::cPacket::operator=(other);
    copy(other);
    return *this;
}

void KHOPCAWeight::copy(const KHOPCAWeight& other)
{
    this->w_n = other.w_n;
    this->originatorAddr = other.originatorAddr;
    this->destAddr = other.destAddr;
    this->pow = other.pow;
    this->entryTime = other.entryTime;
    this->tree = other.tree;
}

void KHOPCAWeight::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::omnetpp::cPacket::parsimPack(b);
    doParsimPacking(b,this->w_n);
    doParsimPacking(b,this->originatorAddr);
    doParsimPacking(b,this->destAddr);
    doParsimPacking(b,this->pow);
    doParsimPacking(b,this->entryTime);
    doParsimPacking(b,this->tree);
}

void KHOPCAWeight::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::omnetpp::cPacket::parsimUnpack(b);
    doParsimUnpacking(b,this->w_n);
    doParsimUnpacking(b,this->originatorAddr);
    doParsimUnpacking(b,this->destAddr);
    doParsimUnpacking(b,this->pow);
    doParsimUnpacking(b,this->entryTime);
    doParsimUnpacking(b,this->tree);
}

unsigned int KHOPCAWeight::getW_n() const
{
    return this->w_n;
}

void KHOPCAWeight::setW_n(unsigned int w_n)
{
    this->w_n = w_n;
}

L3Address& KHOPCAWeight::getOriginatorAddr()
{
    return this->originatorAddr;
}

void KHOPCAWeight::setOriginatorAddr(const L3Address& originatorAddr)
{
    this->originatorAddr = originatorAddr;
}

L3Address& KHOPCAWeight::getDestAddr()
{
    return this->destAddr;
}

void KHOPCAWeight::setDestAddr(const L3Address& destAddr)
{
    this->destAddr = destAddr;
}

double KHOPCAWeight::getPow() const
{
    return this->pow;
}

void KHOPCAWeight::setPow(double pow)
{
    this->pow = pow;
}

double KHOPCAWeight::getEntryTime() const
{
    return this->entryTime;
}

void KHOPCAWeight::setEntryTime(double entryTime)
{
    this->entryTime = entryTime;
}

const char * KHOPCAWeight::getTree() const
{
    return this->tree.c_str();
}

void KHOPCAWeight::setTree(const char * tree)
{
    this->tree = tree;
}

class KHOPCAWeightDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    KHOPCAWeightDescriptor();
    virtual ~KHOPCAWeightDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual const char *getFieldDynamicTypeString(void *object, int field, int i) const override;
    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(KHOPCAWeightDescriptor)

KHOPCAWeightDescriptor::KHOPCAWeightDescriptor() : omnetpp::cClassDescriptor("inet::KHOPCAWeight", "omnetpp::cPacket")
{
    propertynames = nullptr;
}

KHOPCAWeightDescriptor::~KHOPCAWeightDescriptor()
{
    delete[] propertynames;
}

bool KHOPCAWeightDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<KHOPCAWeight *>(obj)!=nullptr;
}

const char **KHOPCAWeightDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *KHOPCAWeightDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int KHOPCAWeightDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 6+basedesc->getFieldCount() : 6;
}

unsigned int KHOPCAWeightDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISCOMPOUND,
        FD_ISCOMPOUND,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<6) ? fieldTypeFlags[field] : 0;
}

const char *KHOPCAWeightDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "w_n",
        "originatorAddr",
        "destAddr",
        "pow",
        "entryTime",
        "tree",
    };
    return (field>=0 && field<6) ? fieldNames[field] : nullptr;
}

int KHOPCAWeightDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='w' && strcmp(fieldName, "w_n")==0) return base+0;
    if (fieldName[0]=='o' && strcmp(fieldName, "originatorAddr")==0) return base+1;
    if (fieldName[0]=='d' && strcmp(fieldName, "destAddr")==0) return base+2;
    if (fieldName[0]=='p' && strcmp(fieldName, "pow")==0) return base+3;
    if (fieldName[0]=='e' && strcmp(fieldName, "entryTime")==0) return base+4;
    if (fieldName[0]=='t' && strcmp(fieldName, "tree")==0) return base+5;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *KHOPCAWeightDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "unsigned int",
        "L3Address",
        "L3Address",
        "double",
        "double",
        "string",
    };
    return (field>=0 && field<6) ? fieldTypeStrings[field] : nullptr;
}

const char **KHOPCAWeightDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *KHOPCAWeightDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int KHOPCAWeightDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    KHOPCAWeight *pp = (KHOPCAWeight *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

const char *KHOPCAWeightDescriptor::getFieldDynamicTypeString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldDynamicTypeString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    KHOPCAWeight *pp = (KHOPCAWeight *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}

std::string KHOPCAWeightDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    KHOPCAWeight *pp = (KHOPCAWeight *)object; (void)pp;
    switch (field) {
        case 0: return ulong2string(pp->getW_n());
        case 1: {std::stringstream out; out << pp->getOriginatorAddr(); return out.str();}
        case 2: {std::stringstream out; out << pp->getDestAddr(); return out.str();}
        case 3: return double2string(pp->getPow());
        case 4: return double2string(pp->getEntryTime());
        case 5: return oppstring2string(pp->getTree());
        default: return "";
    }
}

bool KHOPCAWeightDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    KHOPCAWeight *pp = (KHOPCAWeight *)object; (void)pp;
    switch (field) {
        case 0: pp->setW_n(string2ulong(value)); return true;
        case 3: pp->setPow(string2double(value)); return true;
        case 4: pp->setEntryTime(string2double(value)); return true;
        case 5: pp->setTree((value)); return true;
        default: return false;
    }
}

const char *KHOPCAWeightDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        case 1: return omnetpp::opp_typename(typeid(L3Address));
        case 2: return omnetpp::opp_typename(typeid(L3Address));
        default: return nullptr;
    };
}

void *KHOPCAWeightDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    KHOPCAWeight *pp = (KHOPCAWeight *)object; (void)pp;
    switch (field) {
        case 1: return (void *)(&pp->getOriginatorAddr()); break;
        case 2: return (void *)(&pp->getDestAddr()); break;
        default: return nullptr;
    }
}

} // namespace inet

