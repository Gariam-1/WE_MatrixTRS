/**
 * Returns a 4x4 identity matrix as a flat array.
 * @returns {number[]} 4x4 identity matrix
 * @author Gariam
 */
function identityMatrix(){
    return [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
}

/**
 * Checks if the provided matrix is a valid transformation matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix
 * @returns {boolean}
 * @author Gariam
 */
function isMatrixTRS(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = new Float32Array(matrix.m);
    if (!(Array.isArray(matrix) || matrix instanceof Float64Array || matrix instanceof Float32Array)) return false; //check if it's an array
    matrix = new Float32Array(matrix);
    if (matrix.length != 16) return false; //check if it's a 4x4 matrix
    if (matrix[3] != 0 || matrix[7] != 0 || matrix[11] != 0 || matrix[15] != 1) return false; //check if the last row is [0, 0, 0, 1]

    //If the determinant of the submatrix from 00 to 22 is negative or 0 then the matrix is not valid
    const determinant =
        matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
        matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
        matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
    if (determinant <= 0 || isNaN(determinant) || determinant == undefined) return false;

    const transposed = new Float32Array([matrix[0], matrix[4], matrix[8], matrix[1], matrix[5], matrix[9], matrix[2], matrix[6], matrix[10]]);

    const adjugate = new Float32Array([
        matrix[5] * matrix[10] - matrix[6] * matrix[9],
        matrix[2] * matrix[9] - matrix[1] * matrix[10],
        matrix[1] * matrix[6] - matrix[2] * matrix[5],
        matrix[6] * matrix[8] - matrix[4] * matrix[10],
        matrix[0] * matrix[10] - matrix[2] * matrix[8],
        matrix[2] * matrix[4] - matrix[0] * matrix[6],
        matrix[4] * matrix[9] - matrix[5] * matrix[8],
        matrix[1] * matrix[8] - matrix[0] * matrix[9],
        matrix[0] * matrix[5] - matrix[1] * matrix[4]]);
    
    //if the inverse of the submatrix from 00 to 22 is different from the same submatrix transposed then it's not valid
    for (let i = 0; i < adjugate.length; i++){
        const inverse = adjugate[i] / determinant;
        if (inverse != transposed[i]) return false;
    }
    return true;
}

/**
 * Returns a 4x4 transformation matrix as a flat array given the transforms.
 * @param {Vec3} origin
 * @param {Vec3} angles
 * @param {Vec3} scale
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function transformsToMatrixTRS(origin, angles, scale){
    const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
    const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
    const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

    const sinAlpha_sinBeta = sinAlpha * sinBeta;
    const cosAlpha_sinBeta = cosAlpha * sinBeta;

    const m11 = cosGamma * cosBeta * scale.x;
    const m12 = sinGamma * cosBeta * scale.x;
    const m13 = -sinBeta * scale.x;
    const m21 = (cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha) * scale.y;
    const m22 = (sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha) * scale.y;
    const m23 = cosBeta * sinAlpha * scale.y;
    const m31 = (cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha) * scale.z;
    const m32 = (sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha) * scale.z;
    const m33 = cosBeta * cosAlpha * scale.z;

    return [m11, m12, m13, 0, m21, m22, m23, 0, m31, m32, m33, 0, origin.x, origin.y, origin.z, 1];
}

/**
 * Extracts a transformation matrix with only the specified components and returns it.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @param {boolean} origin
 * @param {boolean} angles
 * @param {boolean} scale
 * @returns {number[]} 4x4 transformation matrix with extracted information
 * @author Gariam
 */
function matrixTRSselectTransforms(matrix, origin, angles, scale){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    let newMatrix = new Float64Array(matrix), newScale, currentScale;
    
    const x = new Vec3(matrix[0], matrix[1], matrix[2]).length();
    const y = new Vec3(matrix[4], matrix[5], matrix[6]).length();
    const z = new Vec3(matrix[8], matrix[9], matrix[10]).length();

    if (scale){
        newScale = new Vec3(x, y, z);
        currentScale = newScale;
    } else {
        newScale = new Vec3(1);
        currentScale = new Vec3(x, y, z);
    }

    if (!origin){
        newMatrix[12] = 0;
        newMatrix[13] = 0;
        newMatrix[14] = 0;
    }
    if (!angles){
        newMatrix[0] = newScale.x;
        newMatrix[1] = 0;
        newMatrix[2] = 0;
        newMatrix[4] = 0;
        newMatrix[5] = newScale.y;
        newMatrix[6] = 0;
        newMatrix[8] = 0;
        newMatrix[9] = 0;
        newMatrix[10] = newScale.z;
    } else if (!scale) {
        newMatrix[0] /= currentScale.x;
        newMatrix[1] /= currentScale.x;
        newMatrix[2] /= currentScale.x;
        newMatrix[4] /= currentScale.y;
        newMatrix[5] /= currentScale.y;
        newMatrix[6] /= currentScale.y;
        newMatrix[8] /= currentScale.z;
        newMatrix[9] /= currentScale.z;
        newMatrix[10] /= currentScale.z;
    }

    return [...newMatrix];
}

/**
 * Returns the origin component of the matrix
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} origin vector
 * @author Gariam
 */
function matrixTRStoOrigin(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    return new Vec3(matrix[12], matrix[13], matrix[14]);
}

/**
 * Returns the angles component of the matrix in radians.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} angles vector in radians
 * @author Gariam
 */
function matrixTRStoAngles(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    const right = new Vec3(matrix[0], matrix[1], matrix[2]).normalize();
    const up = new Vec3(matrix[4], matrix[5], matrix[6]).normalize();
    const forward = new Vec3(matrix[8], matrix[9], matrix[10]).normalize();
    let angles = new Vec3();
    
    if (1.0 - Math.abs(right.z) > 1e-6){
        angles.x = Math.atan2(up.z, forward.z);
        angles.y = -Math.asin(right.z);
        angles.z = Math.atan2(right.y, right.x);
    } else {
        if (right.z > 0){
            angles.x = 1.5707963268;
            angles.y = Math.atan2(right.x, up.x);
            angles.z = 0;
        } else {
            angles.x = -1.5707963268;
            angles.y = -Math.atan2(right.x, up.x);
            angles.z = 0;
        }
    }
    return angles;
}

/**
 * Returns the scale component of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} scale vector
 * @author Gariam
 */
function matrixTRStoScale(matrix){
    const x = new Vec3(matrix[0], matrix[1], matrix[2]).length();
    const y = new Vec3(matrix[4], matrix[5], matrix[6]).length();
    const z = new Vec3(matrix[8], matrix[9], matrix[10]).length();
    return new Vec3(x, y, z);
}

/**
 * Returns the right view vector of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} right vector
 * @author Gariam
 */
function matrixTRStoRightVector(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    return new Vec3(matrix[0], matrix[1], matrix[2]).normalize();
}

/**
 * Returns the up view vector of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} up vector
 * @author Gariam
 */
function matrixTRStoUpVector(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    return new Vec3(matrix[4], matrix[5], matrix[6]).normalize();
}

/**
 * Returns the forward view vector of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} forward vector
 * @author Gariam
 */
function matrixTRStoForwardVector(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    return new Vec3(matrix[8], matrix[9], matrix[10]).normalize();
}

/**
 * Returns the longest eigen vector iìof the matrix, null if there are none.
 * 
 * Not translated nor normalized.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {(Vec3 | null)} eigen vector
 * @author Gariam
 */
function matrixTRStoEigenVector(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    const a = matrix[0], b = matrix[1], c = matrix[2];
    const d = matrix[4], e = matrix[5], f = matrix[6];
    const g = matrix[8], h = matrix[9], i = matrix[10];

    //Compute determinant and trace of the matrix
    const det = a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h;
    const trace = a + e + i;

    //Compute eigenvalues using the quadratic formula
    const discriminant = trace * trace - 4 * det;
    if (discriminant < 0) return null;
    const lambda1 = (trace + Math.sqrt(discriminant)) / 2;
    const lambda2 = (trace - Math.sqrt(discriminant)) / 2;

    //Find largest eigenvalue and corresponding eigenvector
    const lambda = (lambda1 > lambda2) ? lambda1 : lambda2;
    let vector = new Vec3(g - i * lambda, h, i - lambda);
    if (Math.abs(vector.x) > Math.abs(vector.y)){
        const temp = vector.x;
        vector.x = vector.y;
        vector.y = temp;
    }
    if (Math.abs(vector.y) > Math.abs(vector.z)){
        const temp = vector.y;
        vector.y = vector.z;
        vector.z = temp;
    }

    return vector;
}

/**
 * Returns a 4x4 transformation matrix translated with the origin vector.
 * If a matrix is provided it sets the matrix with the new translation and returns it.
 * @param {Vec3} origin - origin vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function originToMatrixTRS(origin, matrix){
    if (matrix != undefined){
        if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
        matrix = [...matrix];
        matrix[12] = origin.x;
        matrix[13] = origin.y;
        matrix[14] = origin.z;
        return matrix;
    } else return [1,0,0,0,0,1,0,0,0,0,1,0,origin.x, origin.y, origin.z,1];
}

/**
 * Returns a 4x4 transformation matrix rotated with the angles vector.
 * If a matrix is provided it sets the matrix with the new rotation and returns it.
 * @param {Vec3} angles - angles vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function anglesToMatrixTRS(angles, matrix){
    const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
    const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
    const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

    const sinAlpha_sinBeta = sinAlpha * sinBeta;
    const cosAlpha_sinBeta = cosAlpha * sinBeta;

    const m11 = cosGamma * cosBeta;
    const m12 = sinGamma * cosBeta;
    const m13 = -sinBeta;
    const m21 = cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha;
    const m22 = sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha;
    const m23 = cosBeta * sinAlpha;
    const m31 = cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha;
    const m32 = sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha;
    const m33 = cosBeta * cosAlpha;

    let newMatrix = new Float64Array([m11, m12, m13, 0, m21, m22, m23, 0, m31, m32, m33, 0, 0, 0, 0, 1]);

    if (matrix != undefined){
        if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
        
        const rightLength = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const upLength = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const forwardLength = new Vec3(matrix[8], matrix[9], matrix[10]).length();

        newMatrix[0] *= rightLength;
        newMatrix[1] *= rightLength;
        newMatrix[2] *= rightLength;
        newMatrix[4] *= upLength;
        newMatrix[5] *= upLength;
        newMatrix[6] *= upLength;
        newMatrix[8] *= forwardLength;
        newMatrix[9] *= forwardLength;
        newMatrix[10] *= forwardLength;
        return [...newMatrix];
    } else return [...newMatrix];
}

/**
 * Returns a 4x4 transformation matrix scaled with the scale vector.
 * If a matrix is provided it sets the matrix with the new scale and returns it.
 * @param {Vec3} scale - scale vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function scaleToMatrixTRS(scale, matrix){
    if (matrix != undefined){
        if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;

        const right = new Vec3(matrix[0], matrix[1], matrix[2]).normalize().multiply(scale.x);
        const up = new Vec3(matrix[4], matrix[5], matrix[6]).normalize().multiply(scale.y);
        const forward = new Vec3(matrix[8], matrix[9], matrix[10]).normalize().multiply(scale.z);
        matrix = [...matrix];

        matrix[0] = right.x;
        matrix[1] = right.y;
        matrix[2] = right.z;
        matrix[4] = up.x;
        matrix[5] = up.y;
        matrix[6] = up.z;
        matrix[8] = forward.x;
        matrix[9] = forward.y;
        matrix[10] = forward.z;
    } else return [scale.x,0,0,0,0,scale.y,0,0,0,0,scale.z,0,0,0,0,1];
}

/**
 * Returns a 4x4 transformation matrix with the provided view vectors.
 * If a matrix is provided it sets the matrix with the new vectors and returns it.
 * @param {Vec3} right - right vector
 * @param {Vec3} up - up vector
 * @param {Vec3} forward - forward vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function viewVectorsToMatrixTRS(right, up, forward, matrix){
    newMatrix = new FLoat64Array(matrix);

    newMatrix[0] = right.x;
    newMatrix[1] = right.y;
    newMatrix[2] = right.z;
    newMatrix[4] = up.x;
    newMatrix[5] = up.y;
    newMatrix[6] = up.z;
    newMatrix[8] = forward.x;
    newMatrix[9] = forward.y;
    newMatrix[10] = forward.z;

    if (matrix != undefined){
        if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
        
        const rightLength = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const upLength = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const forwardLength = new Vec3(matrix[8], matrix[9], matrix[10]).length();

        newMatrix[0] *= rightLength;
        newMatrix[1] *= rightLength;
        newMatrix[2] *= rightLength;
        newMatrix[4] *= upLength;
        newMatrix[5] *= upLength;
        newMatrix[6] *= upLength;
        newMatrix[8] *= forwardLength;
        newMatrix[9] *= forwardLength;
        newMatrix[10] *= forwardLength;
        return [...newMatrix];
    } else {
        newMatrix[12] = 0;
        newMatrix[13] = 0;
        newMatrix[14] = 0;
        return [...newMatrix];
    }
}

/**
 * Translates the provided matrix a returns the result
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} translation - translation vector
 * @return {number[]} translated 4x4 transformation matrix
 * @author Gariam
 */
function translateMatrixTRS(matrix, translation){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    matrix = new Float64Array(matrix);
    matrix[12] += translation.x;
    matrix[13] += translation.y;
    matrix[14] += translation.z;
    return [...matrix];
}

/**
 * Rotates the provided matrix a returns the result
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} angles - angles vector
 * @return {number[]} totated 4x4 transformation matrix
 * @author Gariam
 */
function rotateMatrixTRS(matrix, angles){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;

    const rightLength = new Vec3(matrix[0], matrix[1], matrix[2]).length();
    const upLength = new Vec3(matrix[4], matrix[5], matrix[6]).length();
    const forwardLength = new Vec3(matrix[8], matrix[9], matrix[10]).length();
    matrix = new Float64Array(matrix);

    const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
    const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
    const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

    const sinAlpha_sinBeta = sinAlpha * sinBeta;
    const cosAlpha_sinBeta = cosAlpha * sinBeta;

    matrix[0] = cosGamma * cosBeta * rightLength;
    matrix[1] = sinGamma * cosBeta * rightLength;
    matrix[2] = -sinBeta * rightLength;
    matrix[4] = (cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha) * upLength;
    matrix[5] = (sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha) * upLength;
    matrix[6] = cosBeta * sinAlpha * upLength;
    matrix[8] = (cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha) * forwardLength;
    matrix[9] = (sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha) * forwardLength;
    matrix[10] = cosBeta * cosAlpha * forwardLength;

    return [...matrix];
}

/**
 * Scales the provided matrix a returns the result
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} scale - scaling vector
 * @return {number[]} scaled 4x4 transformation matrix
 * @author Gariam
 */
function scaleMatrixTRS(matrix, scale){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    matrix = new Float64Array(matrix);
    matrix[0] *= scale.x;
    matrix[1] *= scale.x;
    matrix[2] *= scale.x;
    matrix[4] *= scale.y;
    matrix[5] *= scale.y;
    matrix[6] *= scale.y;
    matrix[8] *= scale.z;
    matrix[9] *= scale.z;
    matrix[10] *= scale.z;
    return [...matrix];
}

/**
 * Returns the inverse of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {number[]} inverted 4x4 transformation matrix
 * @author Gariam
 */
function inverseMatrixTRS(matrix){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = matrix.m;
    let inverted = [1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
    let right = new Vec3(matrix[0], matrix[1], matrix[2]);
    let up = new Vec3(matrix[4], matrix[5], matrix[6]);
    let forward = new Vec3(matrix[8], matrix[9], matrix[10]);

    //Inverse scale
    right = right.divide(right.lengthSqr());
    up = up.divide(up.lengthSqr());
    forward = forward.divide(forward.lengthSqr());

    //Inverse rotation
    inverted[0] = right.x;
    inverted[1] = up.x;
    inverted[2] = forward.x;
    inverted[4] = right.y;
    inverted[5] = up.y;
    inverted[6] = forward.y;
    inverted[8] = right.z;
    inverted[9] = up.z;
    inverted[10] = forward.z;

    //Inverse translation
    inverted[12] = -matrix[12];
    inverted[13] = -matrix[13];
    inverted[14] = -matrix[14];

    return inverted;
}

/**
 * Multiplies two transformation matrices together and returns the result.
 * @param {(MatrixTRS | Mat4 | number[])} matrixA - 4x4 transformation matrix
 * @param {(MatrixTRS | Mat4 | number[])} matrixB - 4x4 transformation matrix
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function multiplyMatrixTRS(matrixA, matrixB){
    if (matrixA instanceof MatrixTRS || matrixA instanceof Mat4) matrixA = new Float64Array(matrixA.m);
    if (matrixB instanceof MatrixTRS || matrixB instanceof Mat4) matrixB = new Float64Array(matrixB.m);

    const row0 = new Float64Array([matrixB[0], matrixB[1], matrixB[2]]);
    const row1 = new Float64Array([matrixB[4], matrixB[5], matrixB[6]]);
    const row2 = new Float64Array([matrixB[8], matrixB[9], matrixB[10]]);

    const column0 = matrixArrayMultiply(matrixA, row0);
    const column1 = matrixArrayMultiply(matrixA, row1);
    const column2 = matrixArrayMultiply(matrixA, row2);
    const column3 = new Float64Array([matrixA[12] + matrixB[12], matrixA[13] + matrixB[13], matrixA[14] + matrixB[14]]);

    function matrixArrayMultiply(matrix, array){
        const x = array[0] * matrix[0] + array[1] * matrix[4] + array[2] * matrix[8];
        const y = array[0] * matrix[1] + array[1] * matrix[5] + array[2] * matrix[9];
        const z = array[0] * matrix[2] + array[1] * matrix[6] + array[2] * matrix[10];
        return new Float64Array([x, y, z]);
    }

    return [
        column0[0], column0[1], column0[2], 0,
        column1[0], column1[1], column1[2], 0,
        column2[0], column2[1], column2[2], 0,
        column3[0], column3[1], column3[2], 1];
}

/**
 * Multiplies a vector with a matrix and returns the resulting vector.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @param {(Vec3 | Vec2)} vector
 * @returns {Vec3} 3-dimensional vector
 * @author Gariam
 */
function multiplyVectorMatrixTRS(matrix, vector){
    if (matrix instanceof MatrixTRS || matrix instanceof Mat4) matrix = new Float64Array(matrix.m);
    else matrix = new Float64Array(matrix);
    if (vector instanceof Vec2) vector = new Vec3(vector.x, vector.y, 1);
    
    const x = vector.x * matrix[0] + vector.y * matrix[4] + vector.z * matrix[8] + matrix[12];
    const y = vector.x * matrix[1] + vector.y * matrix[5] + vector.z * matrix[9] + matrix[13];
    const z = vector.x * matrix[2] + vector.y * matrix[6] + vector.z * matrix[10] + matrix[14];

    return new Vec3(x, y, z);
}

/**
 * Spherical linear interpolation between the two matrices.
 * @param {(MatrixTRS | Mat4 | number[])} matrixA - 4x4 transformation matrix
 * @param {(MatrixTRS | Mat4 | number[])} matrixB - 4x4 transformation matrix
 * @param {number} value - interpolation factor
 * @param {number=} valueRot - rotation interpolation factor
 * @param {number=} valueScl - scale interpolation factor
 * @returns {number[]}
 * @author Gariam
 */
function slerpMatrixTRS(matrixA, matrixB, value, valueRot, valueScl){
    if (matrixA instanceof MatrixTRS || matrixA instanceof Mat4) matrixA = new Float64Array(matrixA.m);
    else matrixA = new Float64Array(matrixA);
    if (matrixB instanceof MatrixTRS || matrixB instanceof Mat4) matrixB = new Float64Array(matrixB.m);
    else matrixB = new Float64Array(matrixB);
    if (valueRot == undefined) valueRot = value;
    if (valueScl == undefined) valueScl = value;
    if (value == 0 && valueRot == 0 && valueScl == 0) return [...matrixA];
    else if (value == 1 && valueRot == 1 && valueScl == 1) return [...matrixB];
    let matrix;

    function mixVec3(vectorA, vectorB, value){
        vectorA.x = vectorA.x * (1 - value) + vectorB.x * value;
        vectorA.y = vectorA.y * (1 - value) + vectorB.y * value;
        vectorA.z = vectorA.z * (1 - value) + vectorB.z * value;
        return vectorA;
    }

    function matrixToScale(matrix){
        const x = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const y = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const z = new Vec3(matrix[8], matrix[9], matrix[10]).length();
        return new Vec3(x, y, z);
    }

    function matrixToQuaternion(matrix) {
        matrix = new Float64Array(matrix);
        const trace = matrix[0] + matrix[5] + matrix[10];
        let w, x, y, z;

        if (trace > 0) {
            const s = 0.5 / Math.sqrt(trace + 1);
            w = 0.25 / s;
            x = (matrix[9] - matrix[6]) * s;
            y = (matrix[2] - matrix[8]) * s;
            z = (matrix[4] - matrix[1]) * s;
        } else if (matrix[0] > matrix[5] && matrix[0] > matrix[10]) {
            const s = 2 * Math.sqrt(1 + matrix[0] - matrix[5] - matrix[10]);
            w = (matrix[9] - matrix[6]) / s;
            x = 0.25 * s;
            y = (matrix[1] + matrix[4]) / s;
            z = (matrix[2] + matrix[8]) / s;
        } else if (matrix[5] > matrix[10]) {
            const s = 2 * Math.sqrt(1 + matrix[5] - matrix[0] - matrix[10]);
            w = (matrix[2] - matrix[8]) / s;
            x = (matrix[1] + matrix[4]) / s;
            y = 0.25 * s;
            z = (matrix[6] + matrix[9]) / s;
        } else {
            const s = 2 * Math.sqrt(1 + matrix[10] - matrix[0] - matrix[5]);
            w = (matrix[4] - matrix[1]) / s;
            x = (matrix[2] + matrix[8]) / s;
            y = (matrix[6] + matrix[9]) / s;
            z = 0.25 * s;
        }

        const length = Math.sqrt(w * w + x * x + y * y + z * z);
        return new Float64Array([w / length, x / length, y / length, z / length]);
    }

    function slerpQuaternion(q1, q2, t) {
        let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

        if (dot < 0) {
            q2 = new Float64Array(q2.map(value => -value));
            dot = -dot;
        }
        if (dot > 0.9995) {
            return new Float64Array([q1[0] * (1 - t) + q2[0] * t,
                    q1[1] * (1 - t) + q2[1] * t,
                    q1[2] * (1 - t) + q2[2] * t,
                    q1[3] * (1 - t) + q2[3] * t]);
        }

        const theta = Math.acos(dot);
        const sinTheta = Math.sin(theta);
        const w1 = Math.sin((1 - t) * theta) / sinTheta;
        const w2 = Math.sin(t * theta) / sinTheta;

        return new Float64Array([
                q1[0] * w1 + q2[0] * w2,
                q1[1] * w1 + q2[1] * w2,
                q1[2] * w1 + q2[2] * w2,
                q1[3] * w1 + q2[3] * w2]);
    }

    function quaternionToMatrix(q) {
        const w = q[0], x = q[1], y = q[2], z = q[3];
        const xx = x * x, xy = x * y, xz = x * z, xw = x * w;
        const yy = y * y, yz = y * z, yw = y * w;
        const zz = z * z, zw = z * w;
        return [
            1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw), 0,
            2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw), 0,
            2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy), 0,
            0, 0, 0, 1];
    }
    
    //interpolate origin
    const origin = mixVec3(new Vec3(matrixA[12], matrixA[13], matrixA[14]), new Vec3(matrixB[12], matrixB[13], matrixB[14]), value);

    //interpolate angles
    const q1 = matrixToQuaternion(matrixA);
    const q2 = matrixToQuaternion(matrixB);
    const quaternion = slerpQuaternion(q1, q2, valueRot);
    matrix = quaternionToMatrix(quaternion);

    //interpolate scale
    const scaleA = matrixToScale(matrixA);
    const scaleB = matrixToScale(matrixB);
    const scale = mixVec3(scaleA, scaleB, valueScl);

    matrix[0] *= scale.x;
    matrix[1] *= scale.x;
    matrix[2] *= scale.x;
    matrix[4] *= scale.y;
    matrix[5] *= scale.y;
    matrix[6] *= scale.y;
    matrix[8] *= scale.z;
    matrix[9] *= scale.z;
    matrix[10] *= scale.z;
    matrix[12] = origin.x;
    matrix[13] = origin.y;
    matrix[14] = origin.z;

    return matrix;
}