#pragma once

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace records {
struct Json {
 enum Kind { NIL, NUMBER, BOOLEAN, STRING, ARRAY, OBJECT } kind=NIL;
 double number=0; bool boolean=false; std::string string;
 std::vector<Json> array; std::map<std::string,Json> object;
 const Json& at(const std::string& key) const { return object.at(key); }
 const Json& at(size_t i) const { return array.at(i); }
 int integer() const { if(kind!=NUMBER||number!=std::floor(number))throw std::runtime_error("expected integer");return static_cast<int>(number); }
 bool contains(const std::string& key) const { return object.count(key)!=0; }
};
class JsonReader {
 const std::string& s; size_t p=0; const std::set<std::string>& discard;
 [[noreturn]] void fail() const {throw std::runtime_error("invalid JSON at byte "+std::to_string(p));}
 void ws(){while(p<s.size()&&std::isspace(static_cast<unsigned char>(s[p])))++p;}
 bool take(char c){ws();if(p<s.size()&&s[p]==c){++p;return true;}return false;}
 void expect(char c){if(!take(c))fail();}
 std::string quoted(bool keep){expect('"');std::string out;while(p<s.size()){
  char c=s[p++];if(c=='"')return out;if(static_cast<unsigned char>(c)<32)fail();
  if(c=='\\'){if(p==s.size())fail();c=s[p++];if(c=='u'){
   if(p+4>s.size())fail();
   for(int i=0;i<4;++i)if(!std::isxdigit(static_cast<unsigned char>(s[p+i])))fail();
   if(keep)out.append("\\u"+s.substr(p,4));
   p+=4;continue;
  }switch(c){case '"':case '\\':case '/':break;case 'b':c='\b';break;case 'f':c='\f';break;case 'n':c='\n';break;case 'r':c='\r';break;case 't':c='\t';break;default:fail();}}
  if(keep)out.push_back(c);
 }fail();}
 Json value(bool keep){ws();if(p==s.size())fail();Json j;char c=s[p];
  if(c=='{'){++p;j.kind=Json::OBJECT;if(take('}'))return j;do{
   auto k=quoted(true);expect(':');auto v=value(keep&&!discard.count(k));
   if(keep&&!discard.count(k)){if(j.object.count(k))throw std::runtime_error("duplicate JSON key");j.object.emplace(std::move(k),std::move(v));}
  }while(take(','));expect('}');return j;}
  if(c=='['){++p;j.kind=Json::ARRAY;if(take(']'))return j;do{auto v=value(keep);if(keep)j.array.push_back(std::move(v));}while(take(','));expect(']');return j;}
  if(c=='"'){j.kind=Json::STRING;j.string=quoted(keep);return j;}
  if(c=='t'&&s.compare(p,4,"true")==0){p+=4;j.kind=Json::BOOLEAN;j.boolean=true;return j;}
  if(c=='f'&&s.compare(p,5,"false")==0){p+=5;j.kind=Json::BOOLEAN;return j;}
  if(c=='n'&&s.compare(p,4,"null")==0){p+=4;return j;}
  size_t start=p;if(s[p]=='-')++p;if(p==s.size())fail();
  if(s[p]=='0')++p;else{if(s[p]<'1'||s[p]>'9')fail();while(p<s.size()&&std::isdigit(static_cast<unsigned char>(s[p])))++p;}
  if(p<s.size()&&s[p]=='.'){++p;size_t q=p;while(p<s.size()&&std::isdigit(static_cast<unsigned char>(s[p])))++p;if(q==p)fail();}
  if(p<s.size()&&(s[p]=='e'||s[p]=='E')){++p;if(p<s.size()&&(s[p]=='+'||s[p]=='-'))++p;size_t q=p;while(p<s.size()&&std::isdigit(static_cast<unsigned char>(s[p])))++p;if(q==p)fail();}
  j.kind=Json::NUMBER;j.number=std::strtod(s.c_str()+start,nullptr);if(!std::isfinite(j.number))fail();return j;
 }
public:
 JsonReader(const std::string& text,const std::set<std::string>& skip):s(text),discard(skip){}
 Json parse(){auto j=value(true);ws();if(p!=s.size())fail();return j;}
};
inline Json parse_json(const std::string& s,const std::set<std::string>& skip={}){return JsonReader(s,skip).parse();}
}
