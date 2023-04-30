/*
Functions list (29):
    -isTRSMatrix
    -reflectionMatrix
    -projectionMatrix
    -transformsToMatrix
    -selectMatrixTransforms
    -originFromMatrix
    -anglesFromMatrix
    -scaleFromMatrix
    -rightVectorFromMatrix
    -upVectorFromMatrix
    -forwardVectorFromMatrix
    -matrixDeterminant
    -matrixEigenvalues
    -matrixEigenVector
    -originToMatrix
    -anglesToMatrix
    -scaleToMatrix
    -viewVectorsToMatrix
    -matrixLookAt
    -translateMatrix
    -rotateMatrix
    -scaleMatrix
    -inverseMatrix
    -negateMatrix
    -reflectMatrix
    -projectMatrix
    -matrixMultiply
    -vectorMatrixMultiply
    -slerpMatrix
*/

/**
 * Checks if the provided matrix is a valid transformation matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix
 * @returns {boolean}
 * @author Gariam
 */
function isTRSMatrix(matrix){
    if ("m" in matrix) matrix = new Float32Array(matrix.m);
        else matrix = new Float32Array(matrix);

        if (matrix.length != 16) return false; //check if it's a 4x4 matrix
        if (matrix[3] != 0 || matrix[7] != 0 || matrix[11] != 0 || matrix[15] != 1) return false; //check if the last row is [0, 0, 0, 1]

        //repeated operations
        const a = matrix[5] * matrix[10] - matrix[6] * matrix[9];
        const m410 = matrix[4] * matrix[10];
        const m68 = matrix[6] * matrix[8];
        const b = matrix[4] * matrix[9] - matrix[5] * matrix[8];

        //If the determinant of the submatrix from 00 to 22 is negative or 0 then the matrix is not valid
        const determinant = matrix[0] * a - matrix[1] * (m410 - m68) + matrix[2] * b;
        if (determinant <= 0 || isNaN(determinant) || determinant == undefined || determinant == null) return false;

        const transposed = Float32Array.of(matrix[0], matrix[4], matrix[8], matrix[1], matrix[5], matrix[9], matrix[2], matrix[6], matrix[10]);

        const adjugate = Float32Array.of(
            a,
            matrix[2] * matrix[9] - matrix[1] * matrix[10],
            matrix[1] * matrix[6] - matrix[2] * matrix[5],
            m68 - m410,
            matrix[0] * matrix[10] - matrix[2] * matrix[8],
            matrix[2] * matrix[4] - matrix[0] * matrix[6],
            b,
            matrix[1] * matrix[8] - matrix[0] * matrix[9],
            matrix[0] * matrix[5] - matrix[1] * matrix[4]);
        
        //if the inverse of the submatrix from 00 to 22 is different from the same submatrix transposed then it's not valid
        for (let i = 0; i < adjugate.length; i++){
            const inverse = adjugate[i] / determinant;
            if (inverse != transposed[i]) return false;
        }
        return true;
}

/**
 * Returns a 4x4 reflection matrix with the given normal vector.
 * @param {Vec3} normal - normal vector
 * @param {Vec3=} origin - origin of the normal vector
 * @returns {Mat4} reflection matrix
 */
function reflectionMatrix(normal, origin = new Vec3(0)){
    //repeated operations
    const xy = normal.x * normal.y;
    const xz = normal.x * normal.z;
    const yz = normal.y * normal.z;

    //I - 2 * normal * transpose(normal)
    const matrix = Float64Array.of(normal.x * normal.x, xy, xz, 0,
                                xy, normal.y * normal.y, yz, 0,
                                xz, yz, normal.z * normal.z, 0,
                                origin.x, origin.y, origin.z, 1);
    
    matrix[0] = matrix[0] + matrix[0] - 1;
    matrix[1] = matrix[1] + matrix[1];
    matrix[2] = matrix[2] + matrix[2];
    matrix[4] = matrix[4] + matrix[4];
    matrix[5] = matrix[5] + matrix[5] - 1;
    matrix[6] = matrix[6] + matrix[6];
    matrix[8] = matrix[8] + matrix[8];
    matrix[9] = matrix[9] + matrix[9];
    matrix[10] = matrix[10] + matrix[10] - 1;
    matrix[12] = origin.x * (1 - normal.x);
    matrix[13] = origin.y * (1 - normal.y);
    matrix[14] = origin.z * (1 - normal.z);
    
    const mat4 = new Mat4();
    mat4.m = Array.from(matrix);
    return mat4;
}

/**
 * Returns a 4x4 projection matrix with the given normal vector.
 * @param {Vec3} normal - normal vector
 * @param {Vec3=} origin - origin of the normal vector
 * @returns {Mat4} projection matrix
 */
function projectionMatrix(normal, origin = new Vec3(0)){
    //repeated operations
    const xy = normal.x * normal.y;
    const xz = normal.x * normal.z;
    const yz = normal.y * normal.z;

    //I - normal * transpose(normal)
    const matrix = Float64Array.of(1 - normal.x * normal.x, -xy, -xz, 0,
                                -xy, 1 - normal.y * normal.y, -yz, 0,
                                -xz, -yz, 1 - normal.z * normal.z, 0,
                                origin.x, origin.y, origin.z, 1);

    const mat4 = new Mat4();
    mat4.m = Array.from(matrix);
    return mat4;
}

/**
 * Returns a 4x4 transformation matrix as a flat array given the transforms.
 * @param {Vec3} origin - origin vector
 * @param {Vec3} angles - angles vector
 * @param {Vec3} scale - scale vector
 * @returns {number[]} 4x4 transformation matrix
 * @author Gariam
 */
function transformsToMatrix(origin, angles, scale){
    const matrix = new Mat4();
    angles = angles.multiply(0.0174533);
    const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
    const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
    const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

    const sinAlpha_sinBeta = sinAlpha * sinBeta;
    const cosAlpha_sinBeta = cosAlpha * sinBeta;

    matrix.m[0] = cosGamma * cosBeta * scale.x;
    matrix.m[1] = sinGamma * cosBeta * scale.x;
    matrix.m[2] = -sinBeta * scale.x;
    matrix.m[4] = (cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha) * scale.y;
    matrix.m[5] = (sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha) * scale.y;
    matrix.m[6] = cosBeta * sinAlpha * scale.y;
    matrix.m[8] = (cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha) * scale.z;
    matrix.m[9] = (sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha) * scale.z;
    matrix.m[10] = cosBeta * cosAlpha * scale.z;
    matrix.m[12] = origin.x;
    matrix.m[13] = origin.y;
    matrix.m[15] = origin.z;

    return matrix;
}

/**
 * Extracts a transformation matrix with only the specified components and returns it.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @param {boolean} origin
 * @param {boolean} angles
 * @param {boolean} scale
 * @returns {Mat4} 4x4 transformation matrix with extracted information
 * @author Gariam
 */
function selectMatrixTransforms(matrix, origin, angles, scale){
    if ("m" in matrix) matrix = matrix.m;
    let newScale, currentScale;
    const newMatrix = new Mat4();
    newMatrix.m = Array.from(matrix);
    
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
        newMatrix.m[12] = 0;
        newMatrix.m[13] = 0;
        newMatrix.m[14] = 0;
    }
    if (!angles){
        newMatrix.m[0] = newScale.x;
        newMatrix.m[1] = 0;
        newMatrix.m[2] = 0;
        newMatrix.m[4] = 0;
        newMatrix.m[5] = newScale.y;
        newMatrix.m[6] = 0;
        newMatrix.m[8] = 0;
        newMatrix.m[9] = 0;
        newMatrix.m[10] = newScale.z;
    } else if (!scale) {
        newMatrix.m[0] /= currentScale.x;
        newMatrix.m[1] /= currentScale.x;
        newMatrix.m[2] /= currentScale.x;
        newMatrix.m[4] /= currentScale.y;
        newMatrix.m[5] /= currentScale.y;
        newMatrix.m[6] /= currentScale.y;
        newMatrix.m[8] /= currentScale.z;
        newMatrix.m[9] /= currentScale.z;
        newMatrix.m[10] /= currentScale.z;
    }

    return newMatrix;
}

/**
 * Returns the origin component of the matrix
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} origin vector
 * @author Gariam
 */
function originFromMatrix(matrix){
    if ("m" in matrix) matrix = matrix.m;
    return new Vec3(matrix[12], matrix[13], matrix[14]);
}

/**
 * Returns the angles component of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} angles vector
 * @author Gariam
 */
function anglesFromMatrix(matrix){
    if ("m" in matrix) matrix = matrix.m;
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
    return angles.multiply(57.2958);
}

/**
 * Returns the scale component of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} scale vector
 * @author Gariam
 */
function scaleFromMatrix(matrix){
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
function rightVectorFromMatrix(matrix){
    if ("m" in matrix) matrix = matrix.m;
    return new Vec3(matrix[0], matrix[1], matrix[2]).normalize();
}

/**
 * Returns the up view vector of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} up vector
 * @author Gariam
 */
function upVectorFromMatrix(matrix){
    if ("m" in matrix) matrix = matrix.m;
    return new Vec3(matrix[4], matrix[5], matrix[6]).normalize();
}

/**
 * Returns the forward view vector of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Vec3} forward vector
 * @author Gariam
 */
function forwardVectorFromMatrix(matrix){
    if ("m" in matrix) matrix = matrix.m;
    return new Vec3(matrix[8], matrix[9], matrix[10]).normalize();
}

/**
 * Returns the determinant of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {number} determinant
 */
function matrixDeterminant(matrix){
    if ("m" in matrix) matrix = matrix.m;
    //repeated operations
    const v29 = matrix[9] * matrix[2];
    const v213 = matrix[13] * matrix[2];
    
    return matrix[0] * (matrix[5] * matrix[10] - matrix[9] * matrix[6] + matrix[13] * matrix[6] * matrix[10]) -
        matrix[4] * (matrix[1] * matrix[10] - v29 + v213 * matrix[10]) +
        matrix[8] * (matrix[1] * matrix[6] - matrix[5] * matrix[2] + v213 * matrix[6]) -
        matrix[12] * v29 * matrix[6];
}

/**
 * Returns the 3 eigenvalues of the provided matrix, can be null.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {number[]} eigenvalues
 */
function matrixEigenvalues(matrix){
    //repeated operation
    const m510 = matrix[5] * matrix[10];

    //calculate the coefficients of the characteristic polynomial
    const det = matrix[0] * (matrix[5] * matrix[10] - matrix[6] * matrix[9]) -
                matrix[1] * (matrix[4] * matrix[10] - matrix[6] * matrix[8]) +
                matrix[2] * (matrix[4] * matrix[9] - matrix[5] * matrix[8]);
    const c1 = matrix[0] * matrix[5] + matrix[0] * matrix[10] + m510 - matrix[6] * matrix[6] - matrix[2] * matrix[2] - matrix[1] * matrix[1];
    const trace = -(matrix[0] + matrix[5] + matrix[10]);

    //solve the cubic equation
    const p = trace / 3;
    const q = p * p * p + (trace * c1 - det + det + det) / 6;
    const r = c1 / 3;

    const discriminant = (q + q + q) / (p + p) * Math.sqrt(-3 / p);

    const acosDisc = Math.acos(discriminant);
    if (isNaN(acosDisc)) return [null, null, null];
    const sqrtP = Math.sqrt(-p);

    //calculates the 3 eigenvalues
    const pi2 = Math.PI + Math.PI;
    const x1 = sqrtP * Math.cos(acosDisc / 3);
    const x2 = sqrtP * Math.cos((acosDisc + pi2) / 3);
    const x3 = sqrtP * Math.cos((acosDisc + pi2 + pi2) / 3);

    return [x1 + x1 - r, x2 + x2 - r, x3 + x3 - r];
}

/**
 * Returns the eigenvector of the provided matrix corresponding to the provided eigenvalue.
 * 
 * If eigenvalue is null, this function also returns null, while if it's not provided, it defaults to 1.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @param {number=} eigenvalue - eigenvalue
 * @returns {(Vec3 | null)} eigenvector
 */
function matrixEigenvector(eigenvalue = 1){
    if (eigenvalue == null) return null;

    //calculates the matrix A - λI
    const a = matrix[0] - eigenvalue;
    const e = matrix[5] - eigenvalue;
    const i = matrix[10] - eigenvalue;

    // Calculate the nullspace of the matrix A - λI
    const vector = new Vec3();
    if (Math.abs(a) > Math.abs(matrix[4])) {
        if (Math.abs(a) > Math.abs(matrix[8])) {
            vector.y = 1;
            vector.z = 0;
            vector.x = -(matrix[1] * vector.y + matrix[2] * vector.z) / a;
        } else {
            vector.x = 1;
            vector.y = 0;
            vector.z = -(matrix[8] * vector.x + matrix[9] * vector.y) / i;
        }
    } else {
        if (Math.abs(matrix[4]) > Math.abs(matrix[8])) {
            vector.y = 1;
            vector.z = 0;
            vector.x = -(matrix[4] * vector.y + matrix[6] * vector.z) / e;
        } else {
            vector.x = 1;
            vector.y = 0;
            vector.z = -(matrix[8] * vector.x + matrix[9] * vector.y) / i;
        }
    }
    return vector.normalize();
}

/**
 * Returns a 4x4 transformation matrix translated with the origin vector.
 * If a matrix is provided it sets the matrix with the new translation and returns it.
 * @param {Vec3} origin - origin vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {Mat4} 4x4 transformation matrix
 * @author Gariam
 */
function originToMatrix(origin, matrix){
    const newMatrix = new Mat4();
    if (matrix != undefined){
        if ("m" in matrix) newMatrix.m = Array.from(matrix.m);
        else newMatrix.m = Array.from(matrix);

        newMatrix.m[12] = origin.x;
        newMatrix.m[13] = origin.y;
        newMatrix.m[14] = origin.z;
    }
    return newMatrix
}

/**
 * Returns a 4x4 transformation matrix rotated with the angles vector.
 * If a matrix is provided it sets the matrix with the new rotation and returns it.
 * @param {Vec3} angles - angles vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {Mat4} 4x4 transformation matrix
 * @author Gariam
 */
function anglesToMatrix(angles, matrix){
    angles = angles.multiply(0.0174533);
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

    const newMatrix = new Mat4();
    newMatrix.m = [m11, m12, m13, 0, m21, m22, m23, 0, m31, m32, m33, 0, 0, 0, 0, 1];

    if (matrix != undefined){
        if ("m" in matrix) matrix = matrix.m;
        
        const rightLength = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const upLength = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const forwardLength = new Vec3(matrix[8], matrix[9], matrix[10]).length();

        newMatrix.m[0] *= rightLength;
        newMatrix.m[1] *= rightLength;
        newMatrix.m[2] *= rightLength;
        newMatrix.m[4] *= upLength;
        newMatrix.m[5] *= upLength;
        newMatrix.m[6] *= upLength;
        newMatrix.m[8] *= forwardLength;
        newMatrix.m[9] *= forwardLength;
        newMatrix.m[10] *= forwardLength;
        newMatrix.m[12] = matrix[12];
        newMatrix.m[13] = matrix[13];
        newMatrix.m[14] = matrix[14];
    }
    return newMatrix;
}

/**
 * Returns a 4x4 transformation matrix scaled with the scale vector.
 * If a matrix is provided it sets the matrix with the new scale and returns it.
 * @param {Vec3} scale - scale vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {Mat4} 4x4 transformation matrix
 * @author Gariam
 */
function scaleToMatrix(scale, matrix){
    const newMatrix = new Mat4();
    if (matrix != undefined){
        if ("m" in matrix) newMatrix.m = Array.from(matrix.m);
        else newMatrix.m = Array.from(matrix);

        const right = new Vec3(matrix[0], matrix[1], matrix[2]).normalize().multiply(scale.x);
        const up = new Vec3(matrix[4], matrix[5], matrix[6]).normalize().multiply(scale.y);
        const forward = new Vec3(matrix[8], matrix[9], matrix[10]).normalize().multiply(scale.z);

        newMatrix.m[0] = right.x;
        newMatrix.m[1] = right.y;
        newMatrix.m[2] = right.z;
        newMatrix.m[4] = up.x;
        newMatrix.m[5] = up.y;
        newMatrix.m[6] = up.z;
        newMatrix.m[8] = forward.x;
        newMatrix.m[9] = forward.y;
        newMatrix.m[10] = forward.z;
    } else {
        newMatrix.m[0] = scale.x;
        newMatrix.m[5] = scale.y;
        newMatrix.m[10] = scale.z;
    }
    return newMatrix;
}

/**
 * Returns a 4x4 transformation matrix with the provided view vectors.
 * If a matrix is provided it sets the matrix with the new vectors and returns it.
 * @param {Vec3} right - right vector
 * @param {Vec3} up - up vector
 * @param {Vec3} forward - forward vector
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {Mat4} 4x4 transformation matrix
 * @author Gariam
 */
function viewVectorsToMatrix(right, up, forward, matrix){
    const newMatrix = new Mat4();
    newMatrix.m[0] = right.x;
    newMatrix.m[1] = right.y;
    newMatrix.m[2] = right.z;
    newMatrix.m[4] = up.x;
    newMatrix.m[5] = up.y;
    newMatrix.m[6] = up.z;
    newMatrix.m[8] = forward.x;
    newMatrix.m[9] = forward.y;
    newMatrix.m[10] = forward.z;

    if (matrix != undefined){
        if ("m" in matrix) matrix = matrix.m;
        
        const rightLength = new Vec3(matrix[0], matrix[1], matrix[2]).length();
        const upLength = new Vec3(matrix[4], matrix[5], matrix[6]).length();
        const forwardLength = new Vec3(matrix[8], matrix[9], matrix[10]).length();

        newMatrix.m[0] *= rightLength;
        newMatrix.m[1] *= rightLength;
        newMatrix.m[2] *= rightLength;
        newMatrix.m[4] *= upLength;
        newMatrix.m[5] *= upLength;
        newMatrix.m[6] *= upLength;
        newMatrix.m[8] *= forwardLength;
        newMatrix.m[9] *= forwardLength;
        newMatrix.m[10] *= forwardLength;
    }
    return newMatrix;
}

/**
 * Rotates the provaded matrix to make the forward vector point at a specific point in space and returns the resulting matrix.
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} point - point to look at
 * @param {Vec3} upVector - up vector
 * @returns {MatrixTRS} rotated matrix
 */
function matrixLookAt(matrix, point, upVector){
    const newMatrix = new Mat4();
    if ("m" in matrix) newMatrix.m = Array.from(matrix.m);
    else newMatrix.m = Array.from(matrix);

    const forward = this.getOrigin().subtract(point).normalize();
    const right = upVector.cross(forward).normalize();
    const up = forward.cross(right);

    newMatrix.m[0] = right.x;
    newMatrix.m[1] = right.y;
    newMatrix.m[2] = right.z;
    newMatrix.m[4] = up.x;
    newMatrix.m[5] = up.y;
    newMatrix.m[6] = up.z;
    newMatrix.m[8] = forward.x;
    newMatrix.m[9] = forward.y;
    newMatrix.m[10] = forward.z;
    
    return newMatrix;
}

/**
 * Translates the provided matrix a returns the result
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} translation - translation vector
 * @return {Mat4} translated 4x4 transformation matrix
 * @author Gariam
 */
function translateMatrix(matrix, translation){
    const newMatrix = new Mat4();
    if ("m" in matrix) matrix = matrix.m;
    newMatrix.m = Array.from(matrix);
    newMatrix.m[12] += translation.x;
    newMatrix.m[13] += translation.y;
    newMatrix.m[14] += translation.z;
    return newMatrix;
}

/**
 * Rotates the provided matrix a returns the result
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} angles - angles vector
 * @return {Mat4} totated 4x4 transformation matrix
 * @author Gariam
 */
function rotateMatrix(matrix, angles){
    const newMatrix = new Mat4();
    if ("m" in matrix) matrix = matrix.m;

    const rightLength = new Vec3(matrix[0], matrix[1], matrix[2]).length();
    const upLength = new Vec3(matrix[4], matrix[5], matrix[6]).length();
    const forwardLength = new Vec3(matrix[8], matrix[9], matrix[10]).length();
    newMatrix.m = Array.from(matrix);

    angles = angles.multiply(0.0174533);
    const sinAlpha = Math.sin(angles.x), cosAlpha = Math.cos(angles.x);
    const sinBeta = Math.sin(angles.y), cosBeta = Math.cos(angles.y);
    const sinGamma = Math.sin(angles.z), cosGamma = Math.cos(angles.z);

    const sinAlpha_sinBeta = sinAlpha * sinBeta;
    const cosAlpha_sinBeta = cosAlpha * sinBeta;

    newMatrix.m[0] = cosGamma * cosBeta * rightLength;
    newMatrix.m[1] = sinGamma * cosBeta * rightLength;
    newMatrix.m[2] = -sinBeta * rightLength;
    newMatrix.m[4] = (cosGamma * sinAlpha_sinBeta - sinGamma * cosAlpha) * upLength;
    newMatrix.m[5] = (sinGamma * sinAlpha_sinBeta + cosGamma * cosAlpha) * upLength;
    newMatrix.m[6] = cosBeta * sinAlpha * upLength;
    newMatrix.m[8] = (cosGamma * cosAlpha_sinBeta + sinGamma * sinAlpha) * forwardLength;
    newMatrix.m[9] = (sinGamma * cosAlpha_sinBeta - cosGamma * sinAlpha) * forwardLength;
    newMatrix.m[10] = cosBeta * cosAlpha * forwardLength;

    return newMatrix;
}

/**
 * Scales the provided matrix a returns the result
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} scale - scaling vector
 * @return {Mat4} scaled 4x4 transformation matrix
 * @author Gariam
 */
function scaleMatrix(matrix, scale){
    const newMatrix = new Mat4();
    if ("m" in matrix) matrix = matrix.m;
    newMatrix.m = Array.from(matrix);
    newMatrix.m[0] *= scale.x;
    newMatrix.m[1] *= scale.x;
    newMatrix.m[2] *= scale.x;
    newMatrix.m[4] *= scale.y;
    newMatrix.m[5] *= scale.y;
    newMatrix.m[6] *= scale.y;
    newMatrix.m[8] *= scale.z;
    newMatrix.m[9] *= scale.z;
    newMatrix.m[10] *= scale.z;
    return newMatrix;
}

/**
 * Returns the inverse of the matrix.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @returns {Mat4} inverted 4x4 transformation matrix
 * @author Gariam
 */
function inverseMatrix(matrix){
    if ("m" in matrix) matrix = matrix.m;
    let inverted = new Mat4();
    let right = new Vec3(matrix[0], matrix[1], matrix[2]);
    let up = new Vec3(matrix[4], matrix[5], matrix[6]);
    let forward = new Vec3(matrix[8], matrix[9], matrix[10]);

    //Inverse scale
    right = right.divide(right.lengthSqr());
    up = up.divide(up.lengthSqr());
    forward = forward.divide(forward.lengthSqr());

    //Inverse rotation
    inverted.m[0] = right.x;
    inverted.m[1] = up.x;
    inverted.m[2] = forward.x;
    inverted.m[4] = right.y;
    inverted.m[5] = up.y;
    inverted.m[6] = forward.y;
    inverted.m[8] = right.z;
    inverted.m[9] = up.z;
    inverted.m[10] = forward.z;

    //Inverse translation
    inverted.m[12] = -matrix[12];
    inverted.m[13] = -matrix[13];
    inverted.m[14] = -matrix[14];

    return inverted;
}

/**
 * Negates every component of the matrix and returns the result.
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @returns {Mat4} negated matrix
 */
function negateMatrix(matrix){
    const newMatrix = new Mat4();
    if ("m" in matrix) matrix = matrix.m;
    newMatrix.m = Array.from(matrix, value => -value);
    newMatrix.m[15] = 1
    return newMatrix;
}

/**
 * Reflects the matrix on the plane described by the given normal vector and returns the result.
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} normal - normal vector
 * @param {Vec3=} origin - origin of the normal vector
 * @returns {Mat4} reflected matrix
 */
function reflectMatrix(matrix, normal, origin = new Vec3(0)){
    const newMatrix = new Mat4();
    if ("m" in matrix) matrix = new Float64Array(matrixA.m);
    else matrix = new Float64Array(matrix);

    function reflectionMatrix(normal, origin = new Vec3(0)){
        //repeated operations
        const xy = normal.x * normal.y;
        const xz = normal.x * normal.z;
        const yz = normal.y * normal.z;
        //I - 2 * normal * transpose(normal)
        const matrix = Float64Array.of(normal.x * normal.x, xy, xz, 0,
                                    xy, normal.y * normal.y, yz, 0,
                                    xz, yz, normal.z * normal.z, 0,
                                    origin.x, origin.y, origin.z, 1);
        
        matrix[0] = matrix[0] + matrix[0] - 1;
        matrix[1] = matrix[1] + matrix[1];
        matrix[2] = matrix[2] + matrix[2];
        matrix[4] = matrix[4] + matrix[4];
        matrix[5] = matrix[5] + matrix[5] - 1;
        matrix[6] = matrix[6] + matrix[6];
        matrix[8] = matrix[8] + matrix[8];
        matrix[9] = matrix[9] + matrix[9];
        matrix[10] = matrix[10] + matrix[10] - 1;
        matrix[12] = origin.x * (1 - normal.x);
        matrix[13] = origin.y * (1 - normal.y);
        matrix[14] = origin.z * (1 - normal.z);
        return matrix;
    }

    function matrixMultiply(matrixA, matrixB){
        const row0 = Float64Array.of(matrixB[0], matrixB[1], matrixB[2]);
        const row1 = Float64Array.of(matrixB[4], matrixB[5], matrixB[6]);
        const row2 = Float64Array.of(matrixB[8], matrixB[9], matrixB[10]);
    
        const column0 = matrixArrayMultiply(matrixA, row0);
        const column1 = matrixArrayMultiply(matrixA, row1);
        const column2 = matrixArrayMultiply(matrixA, row2);
        const column3 = Float64Array.of(matrixA[12] + matrixB[12], matrixA[13] + matrixB[13], matrixA[14] + matrixB[14]);
    
        function matrixArrayMultiply(matrix, array) {
            const x = array[0] * matrix[0] + array[1] * matrix[4] + array[2] * matrix[8];
            const y = array[0] * matrix[1] + array[1] * matrix[5] + array[2] * matrix[9];
            const z = array[0] * matrix[2] + array[1] * matrix[6] + array[2] * matrix[10];
            return Float64Array.of(x, y, z);
        }

        return [
            column0[0], column0[1], column0[2], 0,
            column1[0], column1[1], column1[2], 0,
            column2[0], column2[1], column2[2], 0,
            column3[0], column3[1], column3[2], 1];
    }

    const reflMatrix = reflectionMatrix(normal, origin);
    newMatrix.m = matrixMultiply(matrix, reflMatrix);
    
    return newMatrix;
}

/**
 * Projects the matrix on the plane described by the given normal vector and returns the result.
 * @param {(MatrixTRS | Mat4 | number[])=} matrix - 4x4 transformation matrix
 * @param {Vec3} normal - normal vector
 * @param {Vec3=} origin - origin of the normal vector
 * @returns {Mat4} projected matrix
 */
function projectMatrix(matrix, normal, origin = new Vec3(0)){
    const newMatrix = new Mat4();
    if ("m" in matrix) matrix = new Float64Array(matrixA.m);
    else matrix = new Float64Array(matrix);

    function projectionMatrix(normal, origin){
        //repeated operations
        const xy = normal.x * normal.y;
        const xz = normal.x * normal.z;
        const yz = normal.y * normal.z;
        //I - normal * transpose(normal)
        return Float64Array.of(1 - normal.x * normal.x, -xy, -xz, 0,
                                    -xy, 1 - normal.y * normal.y, -yz, 0,
                                    -xz, -yz, 1 - normal.z * normal.z, 0,
                                    origin.x, origin.y, origin.z, 1);
    }

    function matrixMultiply(matrixA, matrixB){
        const row0 = Float64Array.of(matrixB[0], matrixB[1], matrixB[2]);
        const row1 = Float64Array.of(matrixB[4], matrixB[5], matrixB[6]);
        const row2 = Float64Array.of(matrixB[8], matrixB[9], matrixB[10]);
    
        const column0 = matrixArrayMultiply(matrixA, row0);
        const column1 = matrixArrayMultiply(matrixA, row1);
        const column2 = matrixArrayMultiply(matrixA, row2);
        const column3 = Float64Array.of(matrixA[12] + matrixB[12], matrixA[13] + matrixB[13], matrixA[14] + matrixB[14]);
    
        function matrixArrayMultiply(matrix, array) {
            const x = array[0] * matrix[0] + array[1] * matrix[4] + array[2] * matrix[8];
            const y = array[0] * matrix[1] + array[1] * matrix[5] + array[2] * matrix[9];
            const z = array[0] * matrix[2] + array[1] * matrix[6] + array[2] * matrix[10];
            return Float64Array.of(x, y, z);
        }

        return [
            column0[0], column0[1], column0[2], 0,
            column1[0], column1[1], column1[2], 0,
            column2[0], column2[1], column2[2], 0,
            column3[0], column3[1], column3[2], 1];
    }

    const projMatrix = projectionMatrix(normal, origin);
    newMatrix.m = matrixMultiply(matrix, projMatrix);
    
    return newMatrix;
}

/**
 * Multiplies two transformation matrices together and returns the result.
 * @param {(MatrixTRS | Mat4 | number[])} matrixA - 4x4 transformation matrix
 * @param {(MatrixTRS | Mat4 | number[])} matrixB - 4x4 transformation matrix
 * @returns {Mat4} 4x4 transformation matrix
 * @author Gariam
 */
function matrixMultiply(matrixA, matrixB){
    if ("m" in matrixA) matrixA = new Float64Array(matrixA.m);
    if ("m" in matrixB) matrixB = new Float64Array(matrixB.m);

    const row0 = Float64Array.of(matrixB[0], matrixB[1], matrixB[2]);
    const row1 = Float64Array.of(matrixB[4], matrixB[5], matrixB[6]);
    const row2 = Float64Array.of(matrixB[8], matrixB[9], matrixB[10]);

    const column0 = matrixArrayMultiply(matrixA, row0);
    const column1 = matrixArrayMultiply(matrixA, row1);
    const column2 = matrixArrayMultiply(matrixA, row2);
    const column3 = Float64Array.of(matrixA[12] + matrixB[12], matrixA[13] + matrixB[13], matrixA[14] + matrixB[14]);

    function matrixArrayMultiply(matrix, array) {
        const x = array[0] * matrix[0] + array[1] * matrix[4] + array[2] * matrix[8];
        const y = array[0] * matrix[1] + array[1] * matrix[5] + array[2] * matrix[9];
        const z = array[0] * matrix[2] + array[1] * matrix[6] + array[2] * matrix[10];
        return Float64Array.of(x, y, z);
    }

    const newMatrix = new Mat4();
    newMatrix.m = [
        column0[0], column0[1], column0[2], 0,
        column1[0], column1[1], column1[2], 0,
        column2[0], column2[1], column2[2], 0,
        column3[0], column3[1], column3[2], 1];
    return newMatrix;
}

/**
 * Multiplies a vector with a matrix and returns the resulting vector.
 * @param {(MatrixTRS | Mat4 | number[])} matrix - 4x4 transformation matrix
 * @param {(Vec3 | Vec2)} vector
 * @returns {Vec3} 3-dimensional vector
 * @author Gariam
 */
function matrixVectorMultiply(matrix, vector){
    if ("m" in matrix) matrix = new Float64Array(matrix.m);
    else matrix = new Float64Array(matrix);
    if (vector instanceof Vec2) vector = new Vec3(vector.x, vector.y, 0);
    
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
 * @returns {Mat4}
 * @author Gariam
 */
function slerpMatrix(matrixA, matrixB, value, valueRot, valueScl){
    if ("m" in matrixA) matrixA = new Float64Array(matrixA.m);
    else matrixA = new Float64Array(matrixA);
    if ("m" in matrixB) matrixB = new Float64Array(matrixB.m);
    else matrixB = new Float64Array(matrixB);
    if (valueRot == undefined) valueRot = value;
    if (valueScl == undefined) valueScl = value;

    const newMatrix = new Mat4();
    if (value == 0 && valueRot == 0 && valueScl == 0) {
        newMatrix.m = Array.from(matrixA);
        return newMatrix;
    } else if (value == 1 && valueRot == 1 && valueScl == 1) {
        newMatrix.m = Array.from(matrixB);
        return newMatrix;
    }

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
        return Float64Array.of(w / length, x / length, y / length, z / length);
    }

    function slerpQuaternion(q1, q2, t) {
        let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];

        if (dot < 0) {
            q2 = Float64Array.from(q2, value => -value);
            dot = -dot;
        }

        if (dot > 0.9995) {
            return Float64Array.of(q1[0] * (1 - t) + q2[0] * t,
                    q1[1] * (1 - t) + q2[1] * t,
                    q1[2] * (1 - t) + q2[2] * t,
                    q1[3] * (1 - t) + q2[3] * t);
        }

        const theta = Math.acos(dot);
        const sinTheta = Math.sin(theta);
        const w1 = Math.sin((1 - t) * theta) / sinTheta;
        const w2 = Math.sin(t * theta) / sinTheta;

        return Float64Array.of(
                q1[0] * w1 + q2[0] * w2,
                q1[1] * w1 + q2[1] * w2,
                q1[2] * w1 + q2[2] * w2,
                q1[3] * w1 + q2[3] * w2);
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
    newMatrix.m = quaternionToMatrix(quaternion);

    //interpolate scale
    const scaleA = matrixToScale(matrixA);
    const scaleB = matrixToScale(matrixB);
    const scale = mixVec3(scaleA, scaleB, valueScl);

    newMatrix.m[0] *= scale.x;
    newMatrix.m[1] *= scale.x;
    newMatrix.m[2] *= scale.x;
    newMatrix.m[4] *= scale.y;
    newMatrix.m[5] *= scale.y;
    newMatrix.m[6] *= scale.y;
    newMatrix.m[8] *= scale.z;
    newMatrix.m[9] *= scale.z;
    newMatrix.m[10] *= scale.z;
    newMatrix.m[12] = origin.x;
    newMatrix.m[13] = origin.y;
    newMatrix.m[14] = origin.z;

    return newMatrix;
}