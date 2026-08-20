#!/bin/bash

echo "=== Checking ffasn1 library for BER decoder ==="
echo

# 1. 라이브러리 찾기
echo "1. Finding ffasn1 library..."
LIB_PATH=$(find /usr/lib /usr/local/lib ~/mcar_v13 -name "*ffasn1*.so*" -o -name "*ffasn1*.a" 2>/dev/null | head -1)

if [ -z "$LIB_PATH" ]; then
    echo "❌ ffasn1 library not found!"
    echo
    echo "Try searching manually:"
    echo "  find /usr -name '*ffasn1*' 2>/dev/null"
    echo "  find ~ -name '*ffasn1*' 2>/dev/null"
    exit 1
fi

echo "✓ Found library: $LIB_PATH"
echo

# 2. 라이브러리 타입 확인
if file "$LIB_PATH" | grep -q "current ar archive"; then
    echo "Library type: Static (.a)"
    TOOL="nm"
elif file "$LIB_PATH" | grep -q "shared object"; then
    echo "Library type: Shared (.so)"
    TOOL="nm"
else
    echo "Unknown library type"
    file "$LIB_PATH"
fi
echo

# 3. BER 관련 심볼 찾기
echo "2. Checking for BER decoder functions..."
echo

# nm 사용 (심볼 테이블 출력)
if command -v nm &> /dev/null; then
    echo "Using 'nm' to check symbols..."
    
    # BER 디코더 함수 확인
    if nm -D "$LIB_PATH" 2>/dev/null | grep -i "asn1_ber_decode"; then
        echo "✅ asn1_ber_decode found!"
    else
        echo "❌ asn1_ber_decode NOT found"
    fi
    
    echo
    
    # BER 인코더 함수 확인
    if nm -D "$LIB_PATH" 2>/dev/null | grep -i "asn1_ber_encode"; then
        echo "✅ asn1_ber_encode found!"
    else
        echo "❌ asn1_ber_encode NOT found"
    fi
    
    echo
    echo "3. All BER-related symbols:"
    nm -D "$LIB_PATH" 2>/dev/null | grep -i "ber" || echo "  (none found)"
    
    echo
    echo "4. All available ASN1 decode functions:"
    nm -D "$LIB_PATH" 2>/dev/null | grep -i "decode" | head -10
    
else
    echo "❌ 'nm' command not found"
fi

echo
echo

# 4. objdump 사용 (더 상세한 정보)
if command -v objdump &> /dev/null; then
    echo "Using 'objdump' to check symbols..."
    
    if objdump -T "$LIB_PATH" 2>/dev/null | grep -i "asn1_ber_decode"; then
        echo "✅ asn1_ber_decode found in objdump!"
    else
        echo "❌ asn1_ber_decode NOT found in objdump"
    fi
else
    echo "Note: 'objdump' not available for additional checking"
fi

echo
echo

# 5. readelf 사용 (ELF 파일 분석)
if command -v readelf &> /dev/null; then
    echo "Using 'readelf' to check symbols..."
    
    if readelf -s "$LIB_PATH" 2>/dev/null | grep -i "asn1_ber_decode"; then
        echo "✅ asn1_ber_decode found in readelf!"
    else
        echo "❌ asn1_ber_decode NOT found in readelf"
    fi
else
    echo "Note: 'readelf' not available for additional checking"
fi

echo
echo "=== Summary ==="
echo "Library: $LIB_PATH"
echo
echo "Available decoders:"
nm -D "$LIB_PATH" 2>/dev/null | grep "asn1.*decode" | awk '{print "  " $3}' | sort -u

echo
echo "=== Manual check commands ==="
echo "  nm -D $LIB_PATH | grep -i ber"
echo "  nm -D $LIB_PATH | grep decode"
echo "  strings $LIB_PATH | grep -i ber"