
#include <quan/min.hpp>
#include <quan/max.hpp>

/*
Test min max overlaods for AP_Param
*/


template <typename T>
void fun ( T t)
{
  volatile  T x = t;
}

int main()
{

   AP_Float x;
   AP_Int16 xx;

   static_assert(quan::is_model_of<quan::AP_Param_Archetype,decltype(x)>::value, "AP_Param archetype failing");
   static_assert(quan::is_model_of<quan::AP_Param_Archetype,decltype(xx)>::value, "AP_Param archetype failing");
  // x = 1;

   float z = max(19.f,19.f);

   auto v = max(x,2.f);
   auto v1 = max(2.f,x);
   auto v2 = min(x,2.f);
   auto v3 = min(2.f,x);

   z = max(19,19.f);

   v = max(xx,2.f);
   v1 = max(2.f,xx);
   v2 = min(xx,2.f);
   v3 = min(2.f,xx);

   fun(z);
   fun(v);
   fun(v1);
   fun(v2);
   fun(v3);

}