import math

def april_tag_interpretation(xa,ya,thetaa,xc,zc,thetac):
    thetar=math.radians(thetaa) +math.radians(thetac) + math.pi + math.atan(xc/zc)
   
    dist = math.sqrt(xc**2 +zc**2)
    xr = xa + math.cos(math.radians(thetaa) + math.radians(thetac))* dist
    yr = ya + math.sin(math.radians(thetaa) + math.radians(thetac))* dist
    return (xr,yr,(thetar*(180/math.pi)) %  (2*math.pi))

if __name__ == "__main__":
    xr,yr,thetar = april_tag_interpretation(10,0,270,1,1.732,30)

    # Verbose
    print(f"xr: {xr}")
    print(f"yr: {yr}")
    print(f"thetar: {thetar}")