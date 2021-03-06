Shader "Custom/ColorFade" {
Properties {
	_ColorBegin ("Color Begin", Color) = (1,1,1,1)
	_ColorEnd ("Color End", Color) = (1,1,1,1)
	_MainTex ("Base (RGB) Trans (A)", 2D) = "white" {}
	_StubTex ("_StubTex (Do not use)", 2D) = "white" {}
}

SubShader {
	Tags {"Queue"="Transparent" "IgnoreProjector"="True" "RenderType"="Transparent"}
	LOD 200

CGPROGRAM
#pragma surface surf Lambert alpha:blend

sampler2D _MainTex;
fixed4 _ColorBegin;
fixed4 _ColorEnd;

struct Input {
	float2 uv_MainTex;
	float2 uv2_StubTex;
	float4 color : COLOR;
};

void surf (Input IN, inout SurfaceOutput o) {
	fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * lerp(_ColorBegin, _ColorEnd, IN.uv2_StubTex.x) * IN.color;
	
	o.Emission = c.rgb;
	o.Alpha = c.a;
}
ENDCG
}


}