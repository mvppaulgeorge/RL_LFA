// Benchmark "adder" written by ABC on Thu Jul 18 01:37:59 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n160, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n180, new_n181,
    new_n182, new_n183, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n196, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n214,
    new_n215, new_n216, new_n217, new_n218, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n307, new_n308, new_n311, new_n312,
    new_n314, new_n315, new_n317, new_n319;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d24x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nanp02aa1n12x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor002aa1d32x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  oa0022aa1n09x5               g006(.a(\a[6] ), .b(\b[5] ), .c(\a[5] ), .d(\b[4] ), .o(new_n102));
  and002aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .o(new_n103));
  nor022aa1n08x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[7] ), .b(\a[8] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  tech160nm_finand02aa1n03p5x5 g011(.a(\b[6] ), .b(\a[7] ), .o1(new_n107));
  nona23aa1d18x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  oai012aa1n02x7               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai013aa1n09x5               g014(.a(new_n109), .b(new_n108), .c(new_n102), .d(new_n103), .o1(new_n110));
  nor002aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  nand02aa1n03x5               g016(.a(\b[0] ), .b(\a[1] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[1] ), .b(\a[2] ), .o1(new_n113));
  aoi012aa1n09x5               g018(.a(new_n111), .b(new_n112), .c(new_n113), .o1(new_n114));
  nor022aa1n16x5               g019(.a(\b[3] ), .b(\a[4] ), .o1(new_n115));
  nanp02aa1n04x5               g020(.a(\b[3] ), .b(\a[4] ), .o1(new_n116));
  nor022aa1n16x5               g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand42aa1n03x5               g022(.a(\b[2] ), .b(\a[3] ), .o1(new_n118));
  nona23aa1n12x5               g023(.a(new_n118), .b(new_n116), .c(new_n115), .d(new_n117), .out0(new_n119));
  oa0012aa1n06x5               g024(.a(new_n116), .b(new_n117), .c(new_n115), .o(new_n120));
  oabi12aa1n18x5               g025(.a(new_n120), .b(new_n114), .c(new_n119), .out0(new_n121));
  xnrc02aa1n02x5               g026(.a(\b[5] ), .b(\a[6] ), .out0(new_n122));
  tech160nm_fixnrc02aa1n04x5   g027(.a(\b[4] ), .b(\a[5] ), .out0(new_n123));
  nor043aa1n06x5               g028(.a(new_n108), .b(new_n122), .c(new_n123), .o1(new_n124));
  nanp02aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n100), .out0(new_n126));
  aoai13aa1n06x5               g031(.a(new_n126), .b(new_n110), .c(new_n121), .d(new_n124), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n99), .b(new_n127), .c(new_n101), .out0(\s[10] ));
  norp02aa1n02x5               g033(.a(new_n100), .b(new_n97), .o1(new_n129));
  aoi022aa1n03x5               g034(.a(new_n127), .b(new_n129), .c(\b[9] ), .d(\a[10] ), .o1(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv000aa1d42x5               g036(.a(\a[12] ), .o1(new_n132));
  inv040aa1d32x5               g037(.a(\a[11] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(\b[10] ), .o1(new_n134));
  oaoi03aa1n02x5               g039(.a(new_n133), .b(new_n134), .c(new_n130), .o1(new_n135));
  xorb03aa1n02x5               g040(.a(new_n135), .b(\b[11] ), .c(new_n132), .out0(\s[12] ));
  xorc02aa1n02x5               g041(.a(\a[11] ), .b(\b[10] ), .out0(new_n137));
  xorc02aa1n02x5               g042(.a(\a[12] ), .b(\b[11] ), .out0(new_n138));
  nano23aa1n06x5               g043(.a(new_n97), .b(new_n100), .c(new_n125), .d(new_n98), .out0(new_n139));
  and003aa1n02x5               g044(.a(new_n139), .b(new_n138), .c(new_n137), .o(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n110), .c(new_n121), .d(new_n124), .o1(new_n141));
  nanp02aa1n03x5               g046(.a(new_n134), .b(new_n133), .o1(new_n142));
  inv000aa1d42x5               g047(.a(\b[11] ), .o1(new_n143));
  nand02aa1n02x5               g048(.a(new_n143), .b(new_n132), .o1(new_n144));
  oai012aa1n18x5               g049(.a(new_n98), .b(new_n100), .c(new_n97), .o1(new_n145));
  oai022aa1d18x5               g050(.a(new_n133), .b(new_n134), .c(new_n143), .d(new_n132), .o1(new_n146));
  aoai13aa1n12x5               g051(.a(new_n144), .b(new_n146), .c(new_n145), .d(new_n142), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  xorc02aa1n12x5               g053(.a(\a[13] ), .b(\b[12] ), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n141), .c(new_n148), .out0(\s[13] ));
  nor042aa1n03x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  inv040aa1n03x5               g056(.a(new_n151), .o1(new_n152));
  aob012aa1n02x5               g057(.a(new_n149), .b(new_n141), .c(new_n148), .out0(new_n153));
  xorc02aa1n12x5               g058(.a(\a[14] ), .b(\b[13] ), .out0(new_n154));
  xnbna2aa1n03x5               g059(.a(new_n154), .b(new_n153), .c(new_n152), .out0(\s[14] ));
  nanp02aa1n02x5               g060(.a(new_n154), .b(new_n149), .o1(new_n156));
  oao003aa1n02x5               g061(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .carry(new_n157));
  aoai13aa1n04x5               g062(.a(new_n157), .b(new_n156), .c(new_n141), .d(new_n148), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanp02aa1n04x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nor042aa1n06x5               g066(.a(\b[15] ), .b(\a[16] ), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  nanp02aa1n04x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  aoi122aa1n02x5               g069(.a(new_n160), .b(new_n163), .c(new_n164), .d(new_n158), .e(new_n161), .o1(new_n165));
  aoi012aa1n02x5               g070(.a(new_n160), .b(new_n158), .c(new_n161), .o1(new_n166));
  nano22aa1n02x4               g071(.a(new_n166), .b(new_n163), .c(new_n164), .out0(new_n167));
  norp02aa1n02x5               g072(.a(new_n167), .b(new_n165), .o1(\s[16] ));
  nano23aa1n06x5               g073(.a(new_n160), .b(new_n162), .c(new_n164), .d(new_n161), .out0(new_n169));
  nand23aa1n03x5               g074(.a(new_n169), .b(new_n149), .c(new_n154), .o1(new_n170));
  nano32aa1n03x7               g075(.a(new_n170), .b(new_n139), .c(new_n138), .d(new_n137), .out0(new_n171));
  aoai13aa1n12x5               g076(.a(new_n171), .b(new_n110), .c(new_n121), .d(new_n124), .o1(new_n172));
  inv000aa1n02x5               g077(.a(new_n170), .o1(new_n173));
  inv000aa1d42x5               g078(.a(new_n160), .o1(new_n174));
  nanp02aa1n02x5               g079(.a(new_n164), .b(new_n161), .o1(new_n175));
  aoai13aa1n04x5               g080(.a(new_n163), .b(new_n175), .c(new_n157), .d(new_n174), .o1(new_n176));
  aoi012aa1d18x5               g081(.a(new_n176), .b(new_n173), .c(new_n147), .o1(new_n177));
  nand02aa1d08x5               g082(.a(new_n172), .b(new_n177), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g084(.a(\a[18] ), .o1(new_n180));
  inv000aa1d42x5               g085(.a(\a[17] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\b[16] ), .o1(new_n182));
  oaoi03aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n178), .o1(new_n183));
  xorb03aa1n02x5               g088(.a(new_n183), .b(\b[17] ), .c(new_n180), .out0(\s[18] ));
  xroi22aa1d06x4               g089(.a(new_n181), .b(\b[16] ), .c(new_n180), .d(\b[17] ), .out0(new_n185));
  nand22aa1n06x5               g090(.a(\b[17] ), .b(\a[18] ), .o1(new_n186));
  nona22aa1n02x4               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(new_n187));
  oaib12aa1n06x5               g092(.a(new_n187), .b(\b[17] ), .c(new_n180), .out0(new_n188));
  nor042aa1n06x5               g093(.a(\b[18] ), .b(\a[19] ), .o1(new_n189));
  nand02aa1n06x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  aoai13aa1n06x5               g096(.a(new_n191), .b(new_n188), .c(new_n178), .d(new_n185), .o1(new_n192));
  aoi112aa1n02x5               g097(.a(new_n191), .b(new_n188), .c(new_n178), .d(new_n185), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n192), .b(new_n193), .out0(\s[19] ));
  xnrc02aa1n02x5               g099(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g100(.a(\b[19] ), .b(\a[20] ), .o1(new_n196));
  nand42aa1n06x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  norb02aa1n06x5               g102(.a(new_n197), .b(new_n196), .out0(new_n198));
  nona22aa1n02x5               g103(.a(new_n192), .b(new_n198), .c(new_n189), .out0(new_n199));
  inv000aa1n06x5               g104(.a(new_n189), .o1(new_n200));
  inv040aa1n03x5               g105(.a(new_n198), .o1(new_n201));
  aoi012aa1n02x7               g106(.a(new_n201), .b(new_n192), .c(new_n200), .o1(new_n202));
  norb02aa1n03x4               g107(.a(new_n199), .b(new_n202), .out0(\s[20] ));
  nona23aa1n06x5               g108(.a(new_n185), .b(new_n190), .c(new_n201), .d(new_n189), .out0(new_n204));
  norp02aa1n02x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  aoi013aa1n06x5               g110(.a(new_n205), .b(new_n186), .c(new_n181), .d(new_n182), .o1(new_n206));
  nona23aa1n12x5               g111(.a(new_n197), .b(new_n190), .c(new_n189), .d(new_n196), .out0(new_n207));
  oaoi03aa1n03x5               g112(.a(\a[20] ), .b(\b[19] ), .c(new_n200), .o1(new_n208));
  inv040aa1n03x5               g113(.a(new_n208), .o1(new_n209));
  oai012aa1d24x5               g114(.a(new_n209), .b(new_n207), .c(new_n206), .o1(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  aoai13aa1n06x5               g116(.a(new_n211), .b(new_n204), .c(new_n172), .d(new_n177), .o1(new_n212));
  xorb03aa1n02x5               g117(.a(new_n212), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g118(.a(\b[20] ), .b(\a[21] ), .o1(new_n214));
  xorc02aa1n02x5               g119(.a(\a[21] ), .b(\b[20] ), .out0(new_n215));
  xorc02aa1n02x5               g120(.a(\a[22] ), .b(\b[21] ), .out0(new_n216));
  aoi112aa1n02x5               g121(.a(new_n214), .b(new_n216), .c(new_n212), .d(new_n215), .o1(new_n217));
  aoai13aa1n02x5               g122(.a(new_n216), .b(new_n214), .c(new_n212), .d(new_n215), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n218), .b(new_n217), .out0(\s[22] ));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\a[22] ), .o1(new_n221));
  xroi22aa1d06x4               g126(.a(new_n220), .b(\b[20] ), .c(new_n221), .d(\b[21] ), .out0(new_n222));
  nanb02aa1n02x5               g127(.a(new_n204), .b(new_n222), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\b[21] ), .o1(new_n224));
  oao003aa1n06x5               g129(.a(new_n221), .b(new_n224), .c(new_n214), .carry(new_n225));
  aoi012aa1n02x5               g130(.a(new_n225), .b(new_n210), .c(new_n222), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n223), .c(new_n172), .d(new_n177), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g133(.a(\b[22] ), .b(\a[23] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[24] ), .b(\b[23] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n02x7               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(\s[24] ));
  nano23aa1n02x4               g139(.a(new_n189), .b(new_n196), .c(new_n197), .d(new_n190), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n222), .b(new_n208), .c(new_n235), .d(new_n188), .o1(new_n236));
  inv000aa1n02x5               g141(.a(new_n225), .o1(new_n237));
  inv000aa1d42x5               g142(.a(\a[23] ), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\a[24] ), .o1(new_n239));
  xroi22aa1d04x5               g144(.a(new_n238), .b(\b[22] ), .c(new_n239), .d(\b[23] ), .out0(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  oai022aa1n02x5               g146(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n242));
  oaib12aa1n02x5               g147(.a(new_n242), .b(new_n239), .c(\b[23] ), .out0(new_n243));
  aoai13aa1n04x5               g148(.a(new_n243), .b(new_n241), .c(new_n236), .d(new_n237), .o1(new_n244));
  nano22aa1n02x4               g149(.a(new_n204), .b(new_n222), .c(new_n240), .out0(new_n245));
  xorc02aa1n02x5               g150(.a(\a[25] ), .b(\b[24] ), .out0(new_n246));
  aoai13aa1n06x5               g151(.a(new_n246), .b(new_n244), .c(new_n178), .d(new_n245), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n244), .b(new_n246), .c(new_n178), .d(new_n245), .o1(new_n248));
  norb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(\s[25] ));
  nor042aa1n03x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .out0(new_n251));
  nona22aa1n02x5               g156(.a(new_n247), .b(new_n251), .c(new_n250), .out0(new_n252));
  inv000aa1d42x5               g157(.a(new_n250), .o1(new_n253));
  aobi12aa1n02x7               g158(.a(new_n251), .b(new_n247), .c(new_n253), .out0(new_n254));
  norb02aa1n03x4               g159(.a(new_n252), .b(new_n254), .out0(\s[26] ));
  inv000aa1d42x5               g160(.a(\a[25] ), .o1(new_n256));
  inv040aa1d32x5               g161(.a(\a[26] ), .o1(new_n257));
  xroi22aa1d06x4               g162(.a(new_n256), .b(\b[24] ), .c(new_n257), .d(\b[25] ), .out0(new_n258));
  nano32aa1n03x7               g163(.a(new_n204), .b(new_n258), .c(new_n222), .d(new_n240), .out0(new_n259));
  nand02aa1d06x5               g164(.a(new_n178), .b(new_n259), .o1(new_n260));
  oao003aa1n02x5               g165(.a(\a[26] ), .b(\b[25] ), .c(new_n253), .carry(new_n261));
  aobi12aa1n06x5               g166(.a(new_n261), .b(new_n244), .c(new_n258), .out0(new_n262));
  xorc02aa1n12x5               g167(.a(\a[27] ), .b(\b[26] ), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n262), .c(new_n260), .out0(\s[27] ));
  norp02aa1n02x5               g169(.a(\b[26] ), .b(\a[27] ), .o1(new_n265));
  inv040aa1n03x5               g170(.a(new_n265), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n263), .o1(new_n267));
  tech160nm_fiaoi012aa1n02p5x5 g172(.a(new_n267), .b(new_n262), .c(new_n260), .o1(new_n268));
  xnrc02aa1n12x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  nano22aa1n03x5               g174(.a(new_n268), .b(new_n266), .c(new_n269), .out0(new_n270));
  aobi12aa1n06x5               g175(.a(new_n259), .b(new_n172), .c(new_n177), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n240), .b(new_n225), .c(new_n210), .d(new_n222), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n258), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n261), .b(new_n273), .c(new_n272), .d(new_n243), .o1(new_n274));
  oaih12aa1n02x5               g179(.a(new_n263), .b(new_n274), .c(new_n271), .o1(new_n275));
  tech160nm_fiaoi012aa1n02p5x5 g180(.a(new_n269), .b(new_n275), .c(new_n266), .o1(new_n276));
  norp02aa1n03x5               g181(.a(new_n276), .b(new_n270), .o1(\s[28] ));
  xnrc02aa1n02x5               g182(.a(\b[28] ), .b(\a[29] ), .out0(new_n278));
  norb02aa1n02x5               g183(.a(new_n263), .b(new_n269), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n274), .c(new_n271), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n266), .carry(new_n281));
  tech160nm_fiaoi012aa1n02p5x5 g186(.a(new_n278), .b(new_n280), .c(new_n281), .o1(new_n282));
  inv000aa1n02x5               g187(.a(new_n279), .o1(new_n283));
  tech160nm_fiaoi012aa1n02p5x5 g188(.a(new_n283), .b(new_n262), .c(new_n260), .o1(new_n284));
  nano22aa1n03x5               g189(.a(new_n284), .b(new_n278), .c(new_n281), .out0(new_n285));
  nor002aa1n02x5               g190(.a(new_n282), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n112), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g192(.a(\b[29] ), .b(\a[30] ), .out0(new_n288));
  norb03aa1n12x5               g193(.a(new_n263), .b(new_n278), .c(new_n269), .out0(new_n289));
  oaih12aa1n02x5               g194(.a(new_n289), .b(new_n274), .c(new_n271), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n288), .b(new_n290), .c(new_n291), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n289), .o1(new_n293));
  tech160nm_fiaoi012aa1n02p5x5 g198(.a(new_n293), .b(new_n262), .c(new_n260), .o1(new_n294));
  nano22aa1n03x5               g199(.a(new_n294), .b(new_n288), .c(new_n291), .out0(new_n295));
  norp02aa1n03x5               g200(.a(new_n292), .b(new_n295), .o1(\s[30] ));
  norb02aa1n02x5               g201(.a(new_n289), .b(new_n288), .out0(new_n297));
  oaih12aa1n02x5               g202(.a(new_n297), .b(new_n274), .c(new_n271), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  tech160nm_fiaoi012aa1n05x5   g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  inv000aa1n02x5               g206(.a(new_n297), .o1(new_n302));
  aoi012aa1n06x5               g207(.a(new_n302), .b(new_n262), .c(new_n260), .o1(new_n303));
  nano22aa1n03x7               g208(.a(new_n303), .b(new_n299), .c(new_n300), .out0(new_n304));
  nor002aa1n02x5               g209(.a(new_n301), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g210(.a(new_n114), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  nona22aa1n02x4               g211(.a(new_n118), .b(new_n114), .c(new_n117), .out0(new_n307));
  aoib12aa1n02x5               g212(.a(new_n117), .b(new_n116), .c(new_n115), .out0(new_n308));
  aboi22aa1n03x5               g213(.a(new_n115), .b(new_n121), .c(new_n307), .d(new_n308), .out0(\s[4] ));
  xorb03aa1n02x5               g214(.a(new_n121), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g215(.a(new_n123), .b(new_n121), .out0(new_n311));
  tech160nm_fioai012aa1n03p5x5 g216(.a(new_n311), .b(\b[4] ), .c(\a[5] ), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  norp02aa1n02x5               g218(.a(\b[5] ), .b(\a[6] ), .o1(new_n314));
  aoib12aa1n02x5               g219(.a(new_n314), .b(new_n312), .c(new_n103), .out0(new_n315));
  xnrb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n03x5               g221(.a(\a[7] ), .b(\b[6] ), .c(new_n315), .o1(new_n317));
  xorb03aa1n02x5               g222(.a(new_n317), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g223(.a(new_n110), .b(new_n126), .c(new_n121), .d(new_n124), .o1(new_n319));
  norb02aa1n02x5               g224(.a(new_n127), .b(new_n319), .out0(\s[9] ));
endmodule


