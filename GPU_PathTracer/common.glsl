/**
 * common.glsl
 * Common types and functions used for ray tracing.
 */

const float pi = 3.14159265358979;
const float epsilon = 0.001;

struct Ray {
    vec3 o;     // origin
    vec3 d;     // direction - always set with normalized vector
    float t;    // time, for motion blur
};

Ray createRay(vec3 o, vec3 d, float t)
{
    Ray r;
    r.o = o;
    r.d = d;
    r.t = t;
    return r;
}

Ray createRay(vec3 o, vec3 d)
{
    return createRay(o, d, 0.0);
}

vec3 pointOnRay(Ray r, float t)
{
    return r.o + r.d * t;
}

float gSeed = 0.0;

uint baseHash(uvec2 p)
{
    p = 1103515245U * ((p >> 1U) ^ (p.yx));
    uint h32 = 1103515245U * ((p.x) ^ (p.y>>3U));
    return h32 ^ (h32 >> 16);
}

float hash1(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    return float(n) / float(0xffffffffU);
}

vec2 hash2(inout float seed) {
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1,seed += 0.1)));
    uvec2 rz = uvec2(n, n * 48271U);
    return vec2(rz.xy & uvec2(0x7fffffffU)) / float(0x7fffffff);
}

vec3 hash3(inout float seed)
{
    uint n = baseHash(floatBitsToUint(vec2(seed += 0.1, seed += 0.1)));
    uvec3 rz = uvec3(n, n * 16807U, n * 48271U);
    return vec3(rz & uvec3(0x7fffffffU)) / float(0x7fffffff);
}

float rand(vec2 v)
{
    return fract(sin(dot(v.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 toLinear(vec3 c)
{
    return pow(c, vec3(2.2));
}

vec3 toGamma(vec3 c)
{
    return pow(c, vec3(1.0 / 2.2));
}

vec2 randomInUnitDisk(inout float seed) {
    vec2 h = hash2(seed) * vec2(1.0, 6.28318530718);
    float phi = h.y;
    float r = sqrt(h.x);
	return r * vec2(sin(phi), cos(phi));
}

vec3 randomInUnitSphere(inout float seed)
{
    vec3 h = hash3(seed) * vec3(2.0, 6.28318530718, 1.0) - vec3(1.0, 0.0, 0.0);
    float phi = h.y;
    float r = pow(h.z, 1.0/3.0);
	return r * vec3(sqrt(1.0 - h.x * h.x) * vec2(sin(phi), cos(phi)), h.x);
}

vec3 randomUnitVector(inout float seed) //to be used in diffuse reflections with distribution cosine
{
    return(normalize(randomInUnitSphere(seed)));
}

struct Camera
{
    vec3 eye;
    vec3 u, v, n;
    float width, height;
    float lensRadius;
    float planeDist, focusDist;
    float time0, time1;
};

Camera createCamera(
    vec3 eye,
    vec3 at,
    vec3 worldUp,
    float fovy,
    float aspect,
    float aperture,  //diametro em multiplos do pixel size
    float focusDist,  //focal ratio
    float time0,
    float time1)
{
    Camera cam;
    if(aperture == 0.0) cam.focusDist = 1.0; //pinhole camera then focus in on vis plane
    else cam.focusDist = focusDist;
    vec3 w = eye - at;
    cam.planeDist = length(w);
    cam.height = 2.0 * cam.planeDist * tan(fovy * pi / 180.0 * 0.5);
    cam.width = aspect * cam.height;

    cam.lensRadius = aperture * 0.5 * cam.width / iResolution.x;  //aperture ratio * pixel size; (1 pixel=lente raio 0.5)
    cam.eye = eye;
    cam.n = normalize(w);
    cam.u = normalize(cross(worldUp, cam.n));
    cam.v = cross(cam.n, cam.u);
    cam.time0 = time0;
    cam.time1 = time1;
    return cam;
}

Ray getRay(Camera cam, vec2 pixel_sample)  //rnd pixel_sample viewport coordinates
{
    vec2 ls = cam.lensRadius * randomInUnitDisk(gSeed);  //ls - lens sample for DOF
    float time = cam.time0 + hash1(gSeed) * (cam.time1 - cam.time0);

    //Calculate eye_offset and ray direction
    vec3 ps = vec3(cam.width * (pixel_sample.x / iResolution.x - 0.5), cam.height * (pixel_sample.y / iResolution.y - 0.5), -cam.planeDist);
    vec3 p = ps * cam.focusDist;
    vec3 ray_direction = cam.u * (p.x - ls.x) + cam.v * (p.y - ls.y) + cam.n * p.z;
    vec3 eye_offset = cam.eye + cam.u * ls.x + cam.v * ls.y;

    return createRay(eye_offset, normalize(ray_direction), time);
}

// MT_ material type
#define MT_DIFFUSE 0
#define MT_METAL 1
#define MT_DIALECTRIC 2

struct Material
{
    int type;
    vec3 albedo;  //diffuse color
    vec3 specColor;  //the color tint for specular reflections. for metals and opaque dieletrics like coloured glossy plastic
    vec3 emissive; //
    float roughness; // controls roughness for metals. It can be used for rough refractions
    float refIdx; // index of refraction for dialectric
    vec3 refractColor; // absorption for beer's law
};

Material createDiffuseMaterial(vec3 albedo)
{
    Material m;
    m.type = MT_DIFFUSE;
    m.albedo = albedo;
    m.specColor = vec3(0.0);
    m.roughness = 1.0;  //ser usado na iluminação direta
    m.refIdx = 1.0;
    m.refractColor = vec3(0.0);
    m.emissive = vec3(0.0);
    return m;
}

Material createMetalMaterial(vec3 specClr, float roughness)
{
    Material m;
    m.type = MT_METAL;
    m.albedo = vec3(0.0);
    m.specColor = specClr;
    m.roughness = roughness;
    m.emissive = vec3(0.0);
    return m;
}

Material createDialectricMaterial(vec3 refractClr, float refIdx, float roughness)
{
    Material m;
    m.type = MT_DIALECTRIC;
    m.albedo = vec3(0.0);
    m.specColor = vec3(0.04);
    m.refIdx = refIdx;
    m.refractColor = refractClr;  
    m.roughness = roughness;
    m.emissive = vec3(0.0);
    return m;
}

struct HitRecord
{
    vec3 pos;
    vec3 normal;
    float t;            // ray parameter
    Material material;
};


float schlick(float cosine, float refIdx)
{
    float r0 = pow((1.0 - refIdx) / (1.0 + refIdx), 2.0);
    float kr = r0 + (1.0 - r0) * pow(1.0 - cosine, 5.0);
    return kr;
}

bool scatter(Ray rIn, HitRecord rec, out vec3 atten, out Ray rScattered)
{
    vec3 normal;
    if (dot(-rIn.d, rec.normal) < 0.0) {
        normal = -rec.normal;
    }
    else {
        normal = rec.normal;
    }

    if (rec.material.type == MT_DIFFUSE)
    {
        rScattered = createRay(rec.pos + normal * epsilon, normalize(normal + randomUnitVector(gSeed)), rIn.t);
        atten = (rec.material.albedo * max(dot(rScattered.d, rec.normal), 0.0)) / pi;
        return true;
    }
    if (rec.material.type == MT_METAL)
    {
        vec3 reflectedRay = reflect(rIn.d, normal);
        rScattered = createRay(rec.pos + normal * epsilon, normalize(reflectedRay + randomInUnitSphere(gSeed) * rec.material.roughness), rIn.t);
        atten = rec.material.specColor;
        return true;
    }
    if (rec.material.type == MT_DIALECTRIC)
    {
        atten = vec3(1.0);
        float niOverNt;
        float cosine;
        vec3 vt = dot(normal, -rIn.d) * normal + rIn.d;
        float sin_theta_i = length(vt), sin_theta_t;

        if(dot(rIn.d, rec.normal) > 0.0) //hit inside
        {
            niOverNt = rec.material.refIdx;
            sin_theta_t = niOverNt * sin_theta_i;
            cosine = sqrt(1.0 - pow(sin_theta_t, 2.0));
            atten = exp(-rec.material.refractColor * rec.t);
        }
        else  //hit from outside
        {
            niOverNt = 1.0 / rec.material.refIdx;
            sin_theta_t = niOverNt * sin_theta_i;
            cosine = -dot(rIn.d, rec.normal); 
        }

        //Use probabilistic math to decide if scatter a reflected ray or a refracted ray

        float reflectProb;
        
        if (sin_theta_t <= 1.0) {
            reflectProb = schlick(cosine, rec.material.refIdx);
        }
        else {
            reflectProb = 1.0;
        }

        if (hash1(gSeed) < reflectProb) { // Reflection
            vec3 reflectedRay = reflect(rIn.d, normal);
            rScattered = createRay(rec.pos + normal * epsilon, normalize(reflectedRay + randomInUnitSphere(gSeed) * rec.material.roughness), rIn.t);
        }
        else { // Refraction
            vec3 refractedRay = refract(rIn.d, normal, niOverNt);
            rScattered = createRay(rec.pos - normal * epsilon, refractedRay, rIn.t);
        }

        return true;
    }
    return false;
}

struct Triangle {vec3 a; vec3 b; vec3 c; };

Triangle createTriangle(vec3 v0, vec3 v1, vec3 v2)
{
    Triangle t;
    t.a = v0; t.b = v1; t.c = v2;
    return t;
}

bool hit_triangle(Triangle triangle, Ray r, float tmin, float tmax, out HitRecord rec)
{
    vec3 normal = normalize(cross(triangle.b - triangle.a, triangle.c - triangle.a));

    if (dot(normal, r.d) == 0.0) return false;

    float a = triangle.b.x - triangle.a.x;
    float b = triangle.c.x - triangle.a.x;
    float c = -r.d.x;
    float d = r.o.x - triangle.a.x;
    float e = triangle.b.y - triangle.a.y;
    float f = triangle.c.y - triangle.a.y;
    float g = -r.d.y;
    float h = r.o.y - triangle.a.y;
    float i = triangle.b.z - triangle.a.z;
    float j = triangle.c.z - triangle.a.z;
    float k = -r.d.z;
    float l = r.o.z - triangle.a.z;

    float fk_gj = f * k - g * j;
    float gi_ek = g * i - e * k;
    float ej_fi = e * j - f * i;
    float _gl_hk = g * l - h * k;
    float el_hi = e * l - h * i;
    float den = a * fk_gj + b * gi_ek + c * ej_fi;
    float num_beta = d * fk_gj + b * _gl_hk + c * (h * j - f * l);
    float num_gama = a * (-_gl_hk) + d * gi_ek + c * el_hi;
    float num_t = a * (f * l - h * j) + b * (-el_hi) + d * ej_fi;

    float beta = num_beta / den;
    float gama = num_gama / den;
    float t = num_t / den;

    if(t < tmax && t > tmin && 0.0 <= beta && 0.0 <= gama && beta + gama <= 1.0)
    {
        rec.t = t;
        rec.normal = normal;
        rec.pos = pointOnRay(r, rec.t);
        return true;
    }
    return false;
}


struct Sphere
{
    vec3 center;
    float radius;
};

Sphere createSphere(vec3 center, float radius)
{
    Sphere s;
    s.center = center;
    s.radius = radius;
    return s;
}


struct MovingSphere
{
    vec3 center0, center1;
    float radius;
    float time0, time1;
};

MovingSphere createMovingSphere(vec3 center0, vec3 center1, float radius, float time0, float time1)
{
    MovingSphere s;
    s.center0 = center0;
    s.center1 = center1;
    s.radius = radius;
    s.time0 = time0;
    s.time1 = time1;
    return s;
}

vec3 center(MovingSphere mvsphere, float time)
{
    return mvsphere.center0 + ((time - mvsphere.time0) / (mvsphere.time1 - mvsphere.time0)) * (mvsphere.center1 - mvsphere.center0);
}


/*
 * The function naming convention changes with these functions to show that they implement a sort of interface for
 * the book's notion of "hittable". E.g. hit_<type>.
 */

bool hit_sphere(Sphere s, Ray r, float tmin, float tmax, out HitRecord rec)
{
    vec3 oc = s.center - r.o;
    float b = dot(r.d, oc);
    float c = dot(oc, oc) - (s.radius * s.radius);
    float t;

    if (c > 0.0) {
        if (b <= 0.0) return false;
        float discriminant = b * b - c;
        if (discriminant <= 0.0) return false;
        t = b - sqrt(discriminant);
    }
    else {
        float discriminant = b * b - c;
        t = b + sqrt(discriminant);
    }
	
    if(t < tmax && t > tmin) {
        rec.t = t;
        rec.pos = pointOnRay(r, rec.t);
        rec.normal = normalize(rec.pos - s.center);
        if (s.radius < 0.0) rec.normal *= -1.0;
        return true;
    }
    else return false;
}

bool hit_movingSphere(MovingSphere s, Ray r, float tmin, float tmax, out HitRecord rec)
{
    vec3 center = center(s, r.t);
    vec3 oc = center - r.o;
    float b = dot(r.d, oc);
    float c = dot(oc, oc) - (s.radius * s.radius);
    float t;

    if (c > 0.0) {
        if (b <= 0.0) return false;
        float discriminant = b * b - c;
        if (discriminant <= 0.0) return false;
        t = b - sqrt(discriminant);
    }
    else {
        float discriminant = b * b - c;
        t = b + sqrt(discriminant);
    }
	
    if(t < tmax && t > tmin) {
        rec.t = t;
        rec.pos = pointOnRay(r, rec.t);
        rec.normal = normalize(rec.pos - center);
        if (s.radius < 0.0) rec.normal *= -1.0;
        return true;
    }
    else return false;
}

struct pointLight {
    vec3 pos;
    vec3 color;
};

pointLight createPointLight(vec3 pos, vec3 color) 
{
    pointLight l;
    l.pos = pos;
    l.color = color;
    return l;
}

struct areaLight {
    int spp;
    vec3 lightSamples[16];
    vec3 color;
    float w;
    float l;
};

areaLight createAreaLight(vec3 origin, float w, float l, vec3 color)
{
    areaLight al;
    al.spp = 16;
    al.color = color;
    al.w = w;
    al.l = l;

    float sqrt_spp = sqrt(float(al.spp));
    int floor_sqrt_spp = int(sqrt_spp);

    for (int p = 0; p < floor_sqrt_spp; p++) {
        for (int q = 0; q < floor_sqrt_spp; q++) {
            al.lightSamples[p * floor_sqrt_spp + q] = vec3(origin.x + float(p) * w / sqrt_spp, origin.y, origin.z + float(q) * l / sqrt_spp);
        }
    }

    return al;
}