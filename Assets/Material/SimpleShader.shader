// Upgrade NOTE: replaced 'mul(UNITY_MATRIX_MVP,*)' with 'UnityObjectToClipPos(*)'

Shader "Custom/SimpleShader"
{
    Properties
    {

    }
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        LOD 100

        Tags { "RenderType"="Transparent" "Queue"="Transparent" }
		Blend SrcAlpha OneMinusSrcAlpha
		ZWrite Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag


            #include "UnityCG.cginc"
        struct v2f{
            float4 vertex:SV_POSITION;
            float4 position:TEXCOORD1;
            float2 uv:TEXCOORD0;
        };
        v2f vert(appdata_base v){
            v2f o;
            o.vertex=UnityObjectToClipPos(v.vertex);
            o.position=v.vertex;
            o.uv=v.texcoord;
            return o;
        }
           
        float circle(float2 uv,float2 center){
            float2 offset=uv-center;
            float len=length(offset);
            return step(len,0.2)-step(len,0.19);
        } 
           

            float4 frag (v2f i) : SV_Target
			{
				float2 centreOffset = (i.uv.xy - 0.5) * 2;
				float sqrDst = dot(centreOffset, centreOffset);
				float delta = fwidth(sqrt(sqrDst));
				float alpha = 1 - smoothstep(1 - delta, 1 + delta, sqrDst);

				return float4(65 / 255.0, 105 / 255.0, 250 / 255.0, alpha);
			}
            ENDCG
        }
    }
}
