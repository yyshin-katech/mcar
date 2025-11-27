#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import shapefile
import os

def load_shapefile(shp_file):
    """Load shapefile and return line features"""
    try:
        sf = shapefile.Reader(shp_file)
        print(f"Loading shapefile: {os.path.basename(shp_file)}")
        
        shapes = sf.shapes()
        if not shapes:
            print(f"No shapes found in {shp_file}")
            return []
        
        features = []
        
        for j, shape in enumerate(shapes):
            if len(shape.points) == 0:
                continue
            
            feature = {
                'id': j,
                'points': shape.points  # [(x, y), (x, y), ...]
            }
            features.append(feature)
        
        print(f"Loaded {len(features)} features from {os.path.basename(shp_file)}")
        return features
        
    except Exception as e:
        print(f"Error loading {shp_file}: {str(e)}")
        return []