//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************
// Author: Casey Meekhof cmeekhof@microsoft.com


#include "Shared.hlsl"

cbuffer g_cbMatrices
{
	float4x4 mtxWorld;
	float4x4 mtxViewProj[2];
	float4x4 mtxView[2];
};

cbuffer g_cbLight
{
	float4x4 mtxLightViewProj;
	float4 vLightPosV;
};

LitRasterDataSPS main(InstancedVertex vertex)
{
	LitRasterDataSPS output = (LitRasterDataSPS)0;
	int idx = vertex.instId % 2;

	float4 worldPosition;
	float3 worldNormal;
	CalculateWorldPositionAndNormal(
		vertex,
		mtxWorld,
		worldPosition,
		worldNormal);
	output.projectedPosition = mul(worldPosition, mtxViewProj[idx]);

	CalculateViewAndLightDir_ViewSpace(
		worldPosition,
		worldNormal,
		vLightPosV,
		mtxView[idx],
		output.viewSpaceNormal,
		output.viewSpaceViewDir,
		output.viewSpaceLightDir);

	output.color = vertex.color;
	output.texcoord = vertex.texcoord;

	output.rtvId = idx;

	return output;
}
