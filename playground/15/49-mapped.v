// Benchmark "adder" written by ABC on Wed Jul 17 20:06:22 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n193, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n340, new_n341, new_n343, new_n345, new_n347, new_n349;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[8] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  xnrc02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .out0(new_n100));
  orn002aa1n02x7               g005(.a(\a[2] ), .b(\b[1] ), .o(new_n101));
  nanp02aa1n04x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n06x5               g007(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(new_n103));
  nor002aa1n03x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand02aa1n08x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n03x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb03aa1n03x5               g011(.a(new_n105), .b(new_n104), .c(new_n106), .out0(new_n107));
  aoai13aa1n06x5               g012(.a(new_n107), .b(new_n100), .c(new_n103), .d(new_n101), .o1(new_n108));
  nand42aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n06x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand02aa1n06x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nanb03aa1d18x5               g016(.a(new_n110), .b(new_n111), .c(new_n109), .out0(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  inv040aa1d32x5               g018(.a(\a[8] ), .o1(new_n114));
  inv000aa1d42x5               g019(.a(\b[7] ), .o1(new_n115));
  nand02aa1n02x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  nand42aa1n02x5               g021(.a(new_n116), .b(new_n113), .o1(new_n117));
  nor042aa1d18x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor042aa1n03x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nanp02aa1n04x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nona23aa1n09x5               g025(.a(new_n105), .b(new_n120), .c(new_n119), .d(new_n118), .out0(new_n121));
  nor043aa1n06x5               g026(.a(new_n121), .b(new_n117), .c(new_n112), .o1(new_n122));
  norb03aa1n03x5               g027(.a(new_n109), .b(new_n119), .c(new_n118), .out0(new_n123));
  oaoi03aa1n09x5               g028(.a(new_n114), .b(new_n115), .c(new_n110), .o1(new_n124));
  oai013aa1n06x5               g029(.a(new_n124), .b(new_n123), .c(new_n112), .d(new_n117), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n122), .d(new_n108), .o1(new_n127));
  nor022aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n08x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n99), .out0(\s[10] ));
  aoi012aa1n12x5               g036(.a(new_n125), .b(new_n122), .c(new_n108), .o1(new_n132));
  xnrc02aa1n02x5               g037(.a(\b[8] ), .b(\a[9] ), .out0(new_n133));
  oai112aa1n04x5               g038(.a(new_n99), .b(new_n130), .c(new_n132), .d(new_n133), .o1(new_n134));
  nanp02aa1n04x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nor002aa1d24x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  aoi022aa1n02x5               g042(.a(new_n134), .b(new_n129), .c(new_n137), .d(new_n135), .o1(new_n138));
  nano22aa1n12x5               g043(.a(new_n136), .b(new_n129), .c(new_n135), .out0(new_n139));
  aoi012aa1n02x5               g044(.a(new_n138), .b(new_n134), .c(new_n139), .o1(\s[11] ));
  norp02aa1n04x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1n08x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n06x4               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoai13aa1n02x7               g048(.a(new_n143), .b(new_n136), .c(new_n134), .d(new_n139), .o1(new_n144));
  aoi112aa1n02x5               g049(.a(new_n136), .b(new_n143), .c(new_n134), .d(new_n139), .o1(new_n145));
  norb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(\s[12] ));
  inv000aa1n02x5               g051(.a(new_n128), .o1(new_n147));
  nanb03aa1n06x5               g052(.a(new_n136), .b(new_n129), .c(new_n135), .out0(new_n148));
  nano23aa1n03x7               g053(.a(new_n133), .b(new_n148), .c(new_n143), .d(new_n147), .out0(new_n149));
  aoai13aa1n06x5               g054(.a(new_n149), .b(new_n125), .c(new_n122), .d(new_n108), .o1(new_n150));
  oai112aa1n04x5               g055(.a(new_n147), .b(new_n129), .c(\b[8] ), .d(\a[9] ), .o1(new_n151));
  tech160nm_fiaoi012aa1n05x5   g056(.a(new_n141), .b(new_n136), .c(new_n142), .o1(new_n152));
  inv000aa1n02x5               g057(.a(new_n152), .o1(new_n153));
  aoi013aa1n06x4               g058(.a(new_n153), .b(new_n151), .c(new_n139), .d(new_n143), .o1(new_n154));
  nanp02aa1n02x5               g059(.a(new_n150), .b(new_n154), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g061(.a(\a[13] ), .o1(new_n157));
  inv000aa1d42x5               g062(.a(\b[12] ), .o1(new_n158));
  oaoi03aa1n02x5               g063(.a(new_n157), .b(new_n158), .c(new_n155), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norp02aa1n04x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nand42aa1n02x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  nona23aa1n09x5               g069(.a(new_n164), .b(new_n162), .c(new_n161), .d(new_n163), .out0(new_n165));
  aoai13aa1n06x5               g070(.a(new_n164), .b(new_n163), .c(new_n157), .d(new_n158), .o1(new_n166));
  aoai13aa1n04x5               g071(.a(new_n166), .b(new_n165), .c(new_n150), .d(new_n154), .o1(new_n167));
  xorb03aa1n02x5               g072(.a(new_n167), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n09x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand42aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  nor042aa1n12x5               g075(.a(\b[15] ), .b(\a[16] ), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n171), .o1(new_n172));
  nand02aa1n03x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  aoi122aa1n02x5               g078(.a(new_n169), .b(new_n172), .c(new_n173), .d(new_n167), .e(new_n170), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n169), .o1(new_n175));
  norb02aa1n06x5               g080(.a(new_n170), .b(new_n169), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n167), .b(new_n176), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n171), .b(new_n173), .out0(new_n178));
  aoi012aa1n02x5               g083(.a(new_n178), .b(new_n177), .c(new_n175), .o1(new_n179));
  nor002aa1n02x5               g084(.a(new_n179), .b(new_n174), .o1(\s[16] ));
  nona23aa1n09x5               g085(.a(new_n173), .b(new_n170), .c(new_n169), .d(new_n171), .out0(new_n181));
  nor002aa1n02x5               g086(.a(new_n181), .b(new_n165), .o1(new_n182));
  nand02aa1n02x5               g087(.a(new_n149), .b(new_n182), .o1(new_n183));
  nanb02aa1n02x5               g088(.a(new_n141), .b(new_n142), .out0(new_n184));
  norp02aa1n02x5               g089(.a(\b[8] ), .b(\a[9] ), .o1(new_n185));
  norb03aa1n02x5               g090(.a(new_n129), .b(new_n185), .c(new_n128), .out0(new_n186));
  oai013aa1n02x5               g091(.a(new_n152), .b(new_n186), .c(new_n148), .d(new_n184), .o1(new_n187));
  nanp02aa1n02x5               g092(.a(new_n169), .b(new_n173), .o1(new_n188));
  oai112aa1n02x5               g093(.a(new_n188), .b(new_n172), .c(new_n181), .d(new_n166), .o1(new_n189));
  aoi012aa1n06x5               g094(.a(new_n189), .b(new_n187), .c(new_n182), .o1(new_n190));
  oai012aa1n12x5               g095(.a(new_n190), .b(new_n132), .c(new_n183), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  tech160nm_fixorc02aa1n02p5x5 g097(.a(\a[3] ), .b(\b[2] ), .out0(new_n193));
  nanp02aa1n02x5               g098(.a(new_n103), .b(new_n101), .o1(new_n194));
  nona22aa1n02x4               g099(.a(new_n105), .b(new_n106), .c(new_n104), .out0(new_n195));
  tech160nm_fiaoi012aa1n04x5   g100(.a(new_n195), .b(new_n194), .c(new_n193), .o1(new_n196));
  nano22aa1n03x7               g101(.a(new_n110), .b(new_n109), .c(new_n111), .out0(new_n197));
  tech160nm_fixorc02aa1n02p5x5 g102(.a(\a[8] ), .b(\b[7] ), .out0(new_n198));
  nano23aa1n03x7               g103(.a(new_n119), .b(new_n118), .c(new_n120), .d(new_n105), .out0(new_n199));
  nand23aa1n02x5               g104(.a(new_n199), .b(new_n197), .c(new_n198), .o1(new_n200));
  inv000aa1d42x5               g105(.a(new_n118), .o1(new_n201));
  oai112aa1n03x5               g106(.a(new_n201), .b(new_n109), .c(\b[4] ), .d(\a[5] ), .o1(new_n202));
  inv000aa1n02x5               g107(.a(new_n124), .o1(new_n203));
  aoi013aa1n06x4               g108(.a(new_n203), .b(new_n197), .c(new_n202), .d(new_n198), .o1(new_n204));
  oai012aa1n12x5               g109(.a(new_n204), .b(new_n200), .c(new_n196), .o1(new_n205));
  nona23aa1n03x5               g110(.a(new_n139), .b(new_n126), .c(new_n184), .d(new_n128), .out0(new_n206));
  nano23aa1n06x5               g111(.a(new_n169), .b(new_n171), .c(new_n173), .d(new_n170), .out0(new_n207));
  nanb02aa1n06x5               g112(.a(new_n165), .b(new_n207), .out0(new_n208));
  nor042aa1n04x5               g113(.a(new_n208), .b(new_n206), .o1(new_n209));
  norb03aa1n03x5               g114(.a(new_n176), .b(new_n166), .c(new_n178), .out0(new_n210));
  nano22aa1n03x7               g115(.a(new_n210), .b(new_n172), .c(new_n188), .out0(new_n211));
  oai012aa1n12x5               g116(.a(new_n211), .b(new_n154), .c(new_n208), .o1(new_n212));
  tech160nm_fiaoi012aa1n05x5   g117(.a(new_n212), .b(new_n205), .c(new_n209), .o1(new_n213));
  oaoi03aa1n02x5               g118(.a(\a[17] ), .b(\b[16] ), .c(new_n213), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g120(.a(\a[17] ), .o1(new_n216));
  inv040aa1d32x5               g121(.a(\a[18] ), .o1(new_n217));
  xroi22aa1d06x4               g122(.a(new_n216), .b(\b[16] ), .c(new_n217), .d(\b[17] ), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n218), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n219));
  oai022aa1d24x5               g124(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n220));
  oaib12aa1n18x5               g125(.a(new_n220), .b(new_n217), .c(\b[17] ), .out0(new_n221));
  nor042aa1n06x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  nand22aa1n04x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  nanb02aa1n02x5               g128(.a(new_n222), .b(new_n223), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n219), .c(new_n221), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv030aa1n02x5               g132(.a(new_n222), .o1(new_n228));
  tech160nm_fiaoi012aa1n03p5x5 g133(.a(new_n224), .b(new_n219), .c(new_n221), .o1(new_n229));
  nor042aa1n04x5               g134(.a(\b[19] ), .b(\a[20] ), .o1(new_n230));
  nanp02aa1n04x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  nanb02aa1n02x5               g136(.a(new_n230), .b(new_n231), .out0(new_n232));
  nano22aa1n03x7               g137(.a(new_n229), .b(new_n228), .c(new_n232), .out0(new_n233));
  inv000aa1d42x5               g138(.a(new_n221), .o1(new_n234));
  aoai13aa1n03x5               g139(.a(new_n225), .b(new_n234), .c(new_n191), .d(new_n218), .o1(new_n235));
  tech160nm_fiaoi012aa1n02p5x5 g140(.a(new_n232), .b(new_n235), .c(new_n228), .o1(new_n236));
  norp02aa1n02x5               g141(.a(new_n236), .b(new_n233), .o1(\s[20] ));
  nona23aa1d18x5               g142(.a(new_n231), .b(new_n223), .c(new_n222), .d(new_n230), .out0(new_n238));
  inv040aa1n08x5               g143(.a(new_n238), .o1(new_n239));
  nand22aa1n09x5               g144(.a(new_n218), .b(new_n239), .o1(new_n240));
  oaoi03aa1n02x5               g145(.a(\a[20] ), .b(\b[19] ), .c(new_n228), .o1(new_n241));
  oabi12aa1n18x5               g146(.a(new_n241), .b(new_n238), .c(new_n221), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[20] ), .b(\a[21] ), .out0(new_n244));
  oaoi13aa1n04x5               g149(.a(new_n244), .b(new_n243), .c(new_n213), .d(new_n240), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n240), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n244), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n247), .b(new_n242), .c(new_n191), .d(new_n246), .o1(new_n248));
  norp02aa1n02x5               g153(.a(new_n245), .b(new_n248), .o1(\s[21] ));
  orn002aa1n02x5               g154(.a(\a[21] ), .b(\b[20] ), .o(new_n250));
  xnrc02aa1n02x5               g155(.a(\b[21] ), .b(\a[22] ), .out0(new_n251));
  nano22aa1n02x4               g156(.a(new_n245), .b(new_n250), .c(new_n251), .out0(new_n252));
  aoai13aa1n03x5               g157(.a(new_n247), .b(new_n242), .c(new_n191), .d(new_n246), .o1(new_n253));
  aoi012aa1n02x7               g158(.a(new_n251), .b(new_n253), .c(new_n250), .o1(new_n254));
  norp02aa1n03x5               g159(.a(new_n254), .b(new_n252), .o1(\s[22] ));
  nor042aa1n02x5               g160(.a(new_n251), .b(new_n244), .o1(new_n256));
  and003aa1n02x5               g161(.a(new_n218), .b(new_n239), .c(new_n256), .o(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n258));
  oao003aa1n02x5               g163(.a(\a[22] ), .b(\b[21] ), .c(new_n250), .carry(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n242), .c(new_n256), .out0(new_n260));
  nor022aa1n16x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  nand02aa1n03x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  xnbna2aa1n03x5               g168(.a(new_n263), .b(new_n258), .c(new_n260), .out0(\s[23] ));
  inv000aa1d42x5               g169(.a(new_n261), .o1(new_n265));
  aobi12aa1n06x5               g170(.a(new_n263), .b(new_n258), .c(new_n260), .out0(new_n266));
  nor042aa1n02x5               g171(.a(\b[23] ), .b(\a[24] ), .o1(new_n267));
  nanp02aa1n03x5               g172(.a(\b[23] ), .b(\a[24] ), .o1(new_n268));
  nanb02aa1n02x5               g173(.a(new_n267), .b(new_n268), .out0(new_n269));
  nano22aa1n02x4               g174(.a(new_n266), .b(new_n265), .c(new_n269), .out0(new_n270));
  oaib12aa1n02x5               g175(.a(new_n259), .b(new_n243), .c(new_n256), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n263), .b(new_n271), .c(new_n191), .d(new_n257), .o1(new_n272));
  aoi012aa1n03x5               g177(.a(new_n269), .b(new_n272), .c(new_n265), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n270), .o1(\s[24] ));
  nona23aa1d18x5               g179(.a(new_n268), .b(new_n262), .c(new_n261), .d(new_n267), .out0(new_n275));
  inv000aa1n02x5               g180(.a(new_n275), .o1(new_n276));
  nano22aa1n03x7               g181(.a(new_n240), .b(new_n256), .c(new_n276), .out0(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n278));
  nona32aa1n09x5               g183(.a(new_n242), .b(new_n275), .c(new_n251), .d(new_n244), .out0(new_n279));
  oaoi03aa1n02x5               g184(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .o1(new_n280));
  oab012aa1n04x5               g185(.a(new_n280), .b(new_n259), .c(new_n275), .out0(new_n281));
  nanp02aa1n06x5               g186(.a(new_n279), .b(new_n281), .o1(new_n282));
  xnrc02aa1n12x5               g187(.a(\b[24] ), .b(\a[25] ), .out0(new_n283));
  aoib12aa1n06x5               g188(.a(new_n283), .b(new_n278), .c(new_n282), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n283), .o1(new_n285));
  aoi112aa1n02x5               g190(.a(new_n285), .b(new_n282), .c(new_n191), .d(new_n277), .o1(new_n286));
  norp02aa1n02x5               g191(.a(new_n284), .b(new_n286), .o1(\s[25] ));
  nor042aa1n06x5               g192(.a(\b[24] ), .b(\a[25] ), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n288), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[25] ), .b(\a[26] ), .out0(new_n290));
  nano22aa1n02x4               g195(.a(new_n284), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n285), .b(new_n282), .c(new_n191), .d(new_n277), .o1(new_n292));
  aoi012aa1n03x5               g197(.a(new_n290), .b(new_n292), .c(new_n289), .o1(new_n293));
  nor002aa1n02x5               g198(.a(new_n293), .b(new_n291), .o1(\s[26] ));
  nor042aa1n09x5               g199(.a(new_n290), .b(new_n283), .o1(new_n295));
  nano32aa1n03x7               g200(.a(new_n240), .b(new_n295), .c(new_n256), .d(new_n276), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n212), .c(new_n205), .d(new_n209), .o1(new_n297));
  oao003aa1n06x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n289), .carry(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  aoi012aa1n12x5               g204(.a(new_n299), .b(new_n282), .c(new_n295), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  xnbna2aa1n03x5               g206(.a(new_n301), .b(new_n300), .c(new_n297), .out0(\s[27] ));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  inv040aa1n03x5               g208(.a(new_n303), .o1(new_n304));
  aobi12aa1n02x7               g209(.a(new_n301), .b(new_n300), .c(new_n297), .out0(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[27] ), .b(\a[28] ), .out0(new_n306));
  nano22aa1n03x5               g211(.a(new_n305), .b(new_n304), .c(new_n306), .out0(new_n307));
  inv000aa1d42x5               g212(.a(new_n295), .o1(new_n308));
  aoai13aa1n06x5               g213(.a(new_n298), .b(new_n308), .c(new_n279), .d(new_n281), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n301), .b(new_n309), .c(new_n191), .d(new_n296), .o1(new_n310));
  tech160nm_fiaoi012aa1n02p5x5 g215(.a(new_n306), .b(new_n310), .c(new_n304), .o1(new_n311));
  norp02aa1n03x5               g216(.a(new_n311), .b(new_n307), .o1(\s[28] ));
  norb02aa1n02x5               g217(.a(new_n301), .b(new_n306), .out0(new_n313));
  aoai13aa1n03x5               g218(.a(new_n313), .b(new_n309), .c(new_n191), .d(new_n296), .o1(new_n314));
  oao003aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n315));
  xnrc02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .out0(new_n316));
  tech160nm_fiaoi012aa1n02p5x5 g221(.a(new_n316), .b(new_n314), .c(new_n315), .o1(new_n317));
  aobi12aa1n02x7               g222(.a(new_n313), .b(new_n300), .c(new_n297), .out0(new_n318));
  nano22aa1n03x5               g223(.a(new_n318), .b(new_n315), .c(new_n316), .out0(new_n319));
  norp02aa1n03x5               g224(.a(new_n317), .b(new_n319), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g226(.a(new_n301), .b(new_n316), .c(new_n306), .out0(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n309), .c(new_n191), .d(new_n296), .o1(new_n323));
  oao003aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .c(new_n315), .carry(new_n324));
  xnrc02aa1n02x5               g229(.a(\b[29] ), .b(\a[30] ), .out0(new_n325));
  tech160nm_fiaoi012aa1n02p5x5 g230(.a(new_n325), .b(new_n323), .c(new_n324), .o1(new_n326));
  aobi12aa1n02x7               g231(.a(new_n322), .b(new_n300), .c(new_n297), .out0(new_n327));
  nano22aa1n03x5               g232(.a(new_n327), .b(new_n324), .c(new_n325), .out0(new_n328));
  norp02aa1n03x5               g233(.a(new_n326), .b(new_n328), .o1(\s[30] ));
  nona32aa1n03x5               g234(.a(new_n301), .b(new_n325), .c(new_n316), .d(new_n306), .out0(new_n330));
  tech160nm_fiaoi012aa1n05x5   g235(.a(new_n330), .b(new_n300), .c(new_n297), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n324), .carry(new_n332));
  xnrc02aa1n02x5               g237(.a(\b[30] ), .b(\a[31] ), .out0(new_n333));
  nano22aa1n03x5               g238(.a(new_n331), .b(new_n332), .c(new_n333), .out0(new_n334));
  inv000aa1d42x5               g239(.a(new_n330), .o1(new_n335));
  aoai13aa1n03x5               g240(.a(new_n335), .b(new_n309), .c(new_n191), .d(new_n296), .o1(new_n336));
  tech160nm_fiaoi012aa1n02p5x5 g241(.a(new_n333), .b(new_n336), .c(new_n332), .o1(new_n337));
  norp02aa1n03x5               g242(.a(new_n337), .b(new_n334), .o1(\s[31] ));
  xnbna2aa1n03x5               g243(.a(new_n193), .b(new_n103), .c(new_n101), .out0(\s[3] ));
  norb02aa1n02x5               g244(.a(new_n105), .b(new_n106), .out0(new_n340));
  aoi012aa1n02x5               g245(.a(new_n104), .b(new_n194), .c(new_n193), .o1(new_n341));
  oai012aa1n02x5               g246(.a(new_n108), .b(new_n341), .c(new_n340), .o1(\s[4] ));
  norb02aa1n02x5               g247(.a(new_n120), .b(new_n119), .out0(new_n343));
  xobna2aa1n03x5               g248(.a(new_n343), .b(new_n108), .c(new_n105), .out0(\s[5] ));
  aoi013aa1n03x5               g249(.a(new_n119), .b(new_n108), .c(new_n105), .d(new_n120), .o1(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n345), .b(new_n109), .c(new_n201), .out0(\s[6] ));
  tech160nm_fioaoi03aa1n03p5x5 g251(.a(\a[6] ), .b(\b[5] ), .c(new_n345), .o1(new_n347));
  xorb03aa1n02x5               g252(.a(new_n347), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g253(.a(new_n110), .b(new_n347), .c(new_n111), .o1(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n113), .c(new_n116), .out0(\s[8] ));
  xorb03aa1n02x5               g255(.a(new_n132), .b(\b[8] ), .c(new_n97), .out0(\s[9] ));
endmodule

