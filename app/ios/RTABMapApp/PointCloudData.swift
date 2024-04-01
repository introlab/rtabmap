//
//  PointCloudData.swift
//  ThreeDScanner
//
//  Created by Steven Roach on 4/3/18.
//  Copyright © 2018 Steven Roach. All rights reserved.
//

import Foundation
import SceneKit

internal struct PointCloud {
    internal var points: [vector_float3] = []
    internal var pointCloudFrameSizes: [Int32] = []
    internal var pointCloudFrameViewpoints: [SCNVector3] = []
}
