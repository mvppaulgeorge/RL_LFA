// Benchmark "adder" written by ABC on Wed Jul 17 18:58:26 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n345, new_n347, new_n349, new_n351, new_n352, new_n353, new_n355,
    new_n356, new_n357, new_n359, new_n360, new_n361, new_n363;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[9] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\b[8] ), .b(new_n97), .out0(new_n98));
  norp02aa1n04x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv030aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  nand02aa1d04x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand22aa1n09x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n03x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  nor022aa1n06x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x7               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor042aa1n06x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nand02aa1d04x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n03x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nand23aa1n03x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  tech160nm_fioai012aa1n04x5   g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor042aa1d18x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nand22aa1n12x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  nor042aa1d18x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand02aa1d08x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nano23aa1n03x5               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nor002aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nanp02aa1n04x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor042aa1n06x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nand02aa1n03x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  nano23aa1n03x5               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nanp02aa1n03x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  norb02aa1n06x5               g027(.a(new_n115), .b(new_n114), .out0(new_n123));
  oai022aa1n09x5               g028(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n124));
  aoi022aa1d24x5               g029(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n125));
  ao0012aa1n03x7               g030(.a(new_n112), .b(new_n114), .c(new_n113), .o(new_n126));
  aoi013aa1n09x5               g031(.a(new_n126), .b(new_n123), .c(new_n124), .d(new_n125), .o1(new_n127));
  aoai13aa1n12x5               g032(.a(new_n127), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n128));
  xorc02aa1n12x5               g033(.a(\a[9] ), .b(\b[8] ), .out0(new_n129));
  nanp02aa1n09x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  xorc02aa1n12x5               g035(.a(\a[10] ), .b(\b[9] ), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n130), .c(new_n98), .out0(\s[10] ));
  oai022aa1d24x5               g037(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  nand02aa1n02x5               g039(.a(new_n130), .b(new_n134), .o1(new_n135));
  nand02aa1n06x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor002aa1d32x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  tech160nm_fiaoi012aa1n04x5   g042(.a(new_n137), .b(\a[10] ), .c(\b[9] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n138), .b(new_n136), .o1(new_n139));
  nanb02aa1n02x5               g044(.a(new_n139), .b(new_n135), .out0(new_n140));
  norb02aa1n02x5               g045(.a(new_n136), .b(new_n137), .out0(new_n141));
  aoi022aa1n02x5               g046(.a(new_n130), .b(new_n134), .c(\b[9] ), .d(\a[10] ), .o1(new_n142));
  oa0012aa1n03x5               g047(.a(new_n140), .b(new_n142), .c(new_n141), .o(\s[11] ));
  inv000aa1d42x5               g048(.a(new_n137), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n139), .c(new_n130), .d(new_n134), .o1(new_n145));
  nor022aa1n08x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand22aa1n09x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  aoib12aa1n02x5               g053(.a(new_n137), .b(new_n147), .c(new_n146), .out0(new_n149));
  aoi022aa1n02x5               g054(.a(new_n140), .b(new_n149), .c(new_n145), .d(new_n148), .o1(\s[12] ));
  nano23aa1n06x5               g055(.a(new_n146), .b(new_n137), .c(new_n147), .d(new_n136), .out0(new_n151));
  nand23aa1d12x5               g056(.a(new_n151), .b(new_n129), .c(new_n131), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n152), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n128), .b(new_n153), .o1(new_n154));
  aoi012aa1n02x5               g059(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n155));
  nona23aa1n03x5               g060(.a(new_n108), .b(new_n105), .c(new_n104), .d(new_n107), .out0(new_n156));
  oai012aa1n06x5               g061(.a(new_n111), .b(new_n156), .c(new_n155), .o1(new_n157));
  nanb02aa1n06x5               g062(.a(new_n122), .b(new_n157), .out0(new_n158));
  aoi022aa1n02x5               g063(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n159));
  nand03aa1n04x5               g064(.a(new_n138), .b(new_n133), .c(new_n159), .o1(new_n160));
  tech160nm_fiaoi012aa1n05x5   g065(.a(new_n146), .b(new_n137), .c(new_n147), .o1(new_n161));
  nand22aa1n09x5               g066(.a(new_n160), .b(new_n161), .o1(new_n162));
  inv000aa1d42x5               g067(.a(new_n162), .o1(new_n163));
  aoai13aa1n04x5               g068(.a(new_n163), .b(new_n152), .c(new_n158), .d(new_n127), .o1(new_n164));
  nor042aa1n06x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nanp02aa1n04x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  nano22aa1n02x4               g072(.a(new_n167), .b(new_n160), .c(new_n161), .out0(new_n168));
  aoi022aa1n02x5               g073(.a(new_n164), .b(new_n167), .c(new_n154), .d(new_n168), .o1(\s[13] ));
  nor042aa1n04x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  nand02aa1n10x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  aoai13aa1n02x5               g077(.a(new_n172), .b(new_n165), .c(new_n164), .d(new_n166), .o1(new_n173));
  aoi112aa1n02x5               g078(.a(new_n165), .b(new_n172), .c(new_n164), .d(new_n167), .o1(new_n174));
  norb02aa1n02x7               g079(.a(new_n173), .b(new_n174), .out0(\s[14] ));
  nano23aa1n06x5               g080(.a(new_n165), .b(new_n170), .c(new_n171), .d(new_n166), .out0(new_n176));
  aoai13aa1n06x5               g081(.a(new_n176), .b(new_n162), .c(new_n128), .d(new_n153), .o1(new_n177));
  aoi012aa1d24x5               g082(.a(new_n170), .b(new_n165), .c(new_n171), .o1(new_n178));
  nor042aa1n12x5               g083(.a(\b[14] ), .b(\a[15] ), .o1(new_n179));
  nand22aa1n02x5               g084(.a(\b[14] ), .b(\a[15] ), .o1(new_n180));
  norb02aa1d21x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  xnbna2aa1n03x5               g086(.a(new_n181), .b(new_n177), .c(new_n178), .out0(\s[15] ));
  inv000aa1d42x5               g087(.a(new_n178), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n181), .b(new_n183), .c(new_n164), .d(new_n176), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n179), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n181), .o1(new_n186));
  aoai13aa1n03x5               g091(.a(new_n185), .b(new_n186), .c(new_n177), .d(new_n178), .o1(new_n187));
  nor042aa1n06x5               g092(.a(\b[15] ), .b(\a[16] ), .o1(new_n188));
  nand42aa1n03x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  norb02aa1n03x4               g094(.a(new_n189), .b(new_n188), .out0(new_n190));
  norp02aa1n02x5               g095(.a(new_n190), .b(new_n179), .o1(new_n191));
  aoi022aa1n02x5               g096(.a(new_n187), .b(new_n190), .c(new_n184), .d(new_n191), .o1(\s[16] ));
  nona23aa1n09x5               g097(.a(new_n171), .b(new_n166), .c(new_n165), .d(new_n170), .out0(new_n193));
  nano22aa1n12x5               g098(.a(new_n193), .b(new_n181), .c(new_n190), .out0(new_n194));
  nanb02aa1n12x5               g099(.a(new_n152), .b(new_n194), .out0(new_n195));
  aoai13aa1n03x5               g100(.a(new_n180), .b(new_n170), .c(new_n165), .d(new_n171), .o1(new_n196));
  aoi022aa1n06x5               g101(.a(new_n196), .b(new_n185), .c(\a[16] ), .d(\b[15] ), .o1(new_n197));
  aoi112aa1n06x5               g102(.a(new_n197), .b(new_n188), .c(new_n194), .d(new_n162), .o1(new_n198));
  aoai13aa1n12x5               g103(.a(new_n198), .b(new_n195), .c(new_n158), .d(new_n127), .o1(new_n199));
  xorc02aa1n02x5               g104(.a(\a[17] ), .b(\b[16] ), .out0(new_n200));
  nano32aa1d12x5               g105(.a(new_n152), .b(new_n190), .c(new_n176), .d(new_n181), .out0(new_n201));
  nand02aa1d04x5               g106(.a(new_n194), .b(new_n162), .o1(new_n202));
  nona32aa1n02x4               g107(.a(new_n202), .b(new_n197), .c(new_n200), .d(new_n188), .out0(new_n203));
  aoi012aa1n02x5               g108(.a(new_n203), .b(new_n128), .c(new_n201), .o1(new_n204));
  aoi012aa1n02x5               g109(.a(new_n204), .b(new_n199), .c(new_n200), .o1(\s[17] ));
  inv000aa1d42x5               g110(.a(\a[17] ), .o1(new_n206));
  nanb02aa1n02x5               g111(.a(\b[16] ), .b(new_n206), .out0(new_n207));
  nona22aa1n09x5               g112(.a(new_n202), .b(new_n197), .c(new_n188), .out0(new_n208));
  aoai13aa1n04x5               g113(.a(new_n200), .b(new_n208), .c(new_n128), .d(new_n201), .o1(new_n209));
  xorc02aa1n02x5               g114(.a(\a[18] ), .b(\b[17] ), .out0(new_n210));
  xnbna2aa1n03x5               g115(.a(new_n210), .b(new_n209), .c(new_n207), .out0(\s[18] ));
  inv000aa1d42x5               g116(.a(\a[18] ), .o1(new_n212));
  xroi22aa1d04x5               g117(.a(new_n206), .b(\b[16] ), .c(new_n212), .d(\b[17] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n208), .c(new_n128), .d(new_n201), .o1(new_n214));
  nor042aa1n02x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  aoi112aa1n09x5               g120(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n216));
  nor042aa1n04x5               g121(.a(new_n216), .b(new_n215), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[19] ), .b(\b[18] ), .out0(new_n218));
  xnbna2aa1n03x5               g123(.a(new_n218), .b(new_n214), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g125(.a(new_n217), .o1(new_n221));
  aoai13aa1n03x5               g126(.a(new_n218), .b(new_n221), .c(new_n199), .d(new_n213), .o1(new_n222));
  nor002aa1d32x5               g127(.a(\b[18] ), .b(\a[19] ), .o1(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  tech160nm_fixnrc02aa1n05x5   g129(.a(\b[18] ), .b(\a[19] ), .out0(new_n225));
  aoai13aa1n03x5               g130(.a(new_n224), .b(new_n225), .c(new_n214), .d(new_n217), .o1(new_n226));
  tech160nm_fixorc02aa1n04x5   g131(.a(\a[20] ), .b(\b[19] ), .out0(new_n227));
  norp02aa1n02x5               g132(.a(new_n227), .b(new_n223), .o1(new_n228));
  aoi022aa1n02x5               g133(.a(new_n226), .b(new_n227), .c(new_n222), .d(new_n228), .o1(\s[20] ));
  nano32aa1n02x4               g134(.a(new_n225), .b(new_n227), .c(new_n200), .d(new_n210), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n208), .c(new_n128), .d(new_n201), .o1(new_n231));
  and002aa1n12x5               g136(.a(\b[19] ), .b(\a[20] ), .o(new_n232));
  oao003aa1n12x5               g137(.a(\a[20] ), .b(\b[19] ), .c(new_n224), .carry(new_n233));
  oai013aa1d12x5               g138(.a(new_n233), .b(new_n217), .c(new_n225), .d(new_n232), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n231), .b(new_n235), .o1(new_n236));
  nor002aa1d32x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nand42aa1n10x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  inv040aa1n02x5               g144(.a(new_n232), .o1(new_n240));
  oai112aa1n06x5               g145(.a(new_n218), .b(new_n240), .c(new_n216), .d(new_n215), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n239), .o1(new_n242));
  and003aa1n02x5               g147(.a(new_n241), .b(new_n242), .c(new_n233), .o(new_n243));
  aoi022aa1n02x5               g148(.a(new_n236), .b(new_n239), .c(new_n231), .d(new_n243), .o1(\s[21] ));
  aoai13aa1n03x5               g149(.a(new_n239), .b(new_n234), .c(new_n199), .d(new_n230), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n237), .o1(new_n246));
  aoai13aa1n03x5               g151(.a(new_n246), .b(new_n242), .c(new_n231), .d(new_n235), .o1(new_n247));
  nor002aa1n16x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  nand02aa1d28x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  aoib12aa1n02x5               g155(.a(new_n237), .b(new_n249), .c(new_n248), .out0(new_n251));
  aoi022aa1n03x5               g156(.a(new_n247), .b(new_n250), .c(new_n245), .d(new_n251), .o1(\s[22] ));
  norb02aa1n02x5               g157(.a(new_n227), .b(new_n225), .out0(new_n253));
  nano23aa1d15x5               g158(.a(new_n237), .b(new_n248), .c(new_n249), .d(new_n238), .out0(new_n254));
  and003aa1n02x5               g159(.a(new_n253), .b(new_n213), .c(new_n254), .o(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n208), .c(new_n128), .d(new_n201), .o1(new_n256));
  aoi012aa1d24x5               g161(.a(new_n248), .b(new_n237), .c(new_n249), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n234), .c(new_n254), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n256), .b(new_n259), .o1(new_n260));
  nor002aa1d32x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  nanp02aa1n12x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norb02aa1n12x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  aoi112aa1n02x5               g168(.a(new_n263), .b(new_n258), .c(new_n234), .d(new_n254), .o1(new_n264));
  aoi022aa1n02x5               g169(.a(new_n260), .b(new_n263), .c(new_n256), .d(new_n264), .o1(\s[23] ));
  inv000aa1n02x5               g170(.a(new_n259), .o1(new_n266));
  aoai13aa1n02x7               g171(.a(new_n263), .b(new_n266), .c(new_n199), .d(new_n255), .o1(new_n267));
  inv040aa1n15x5               g172(.a(new_n261), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n263), .o1(new_n269));
  aoai13aa1n03x5               g174(.a(new_n268), .b(new_n269), .c(new_n256), .d(new_n259), .o1(new_n270));
  xorc02aa1n12x5               g175(.a(\a[24] ), .b(\b[23] ), .out0(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n261), .o1(new_n272));
  aoi022aa1n03x5               g177(.a(new_n270), .b(new_n271), .c(new_n267), .d(new_n272), .o1(\s[24] ));
  nand23aa1n09x5               g178(.a(new_n254), .b(new_n263), .c(new_n271), .o1(new_n274));
  nano32aa1n02x4               g179(.a(new_n274), .b(new_n213), .c(new_n218), .d(new_n227), .out0(new_n275));
  aoai13aa1n04x5               g180(.a(new_n275), .b(new_n208), .c(new_n128), .d(new_n201), .o1(new_n276));
  nanp02aa1n02x5               g181(.a(\b[23] ), .b(\a[24] ), .o1(new_n277));
  nano32aa1n03x7               g182(.a(new_n257), .b(new_n277), .c(new_n268), .d(new_n262), .out0(new_n278));
  oaoi03aa1n03x5               g183(.a(\a[24] ), .b(\b[23] ), .c(new_n268), .o1(new_n279));
  nor042aa1n06x5               g184(.a(new_n278), .b(new_n279), .o1(new_n280));
  aoai13aa1n12x5               g185(.a(new_n280), .b(new_n274), .c(new_n241), .d(new_n233), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  nanp02aa1n02x5               g187(.a(new_n276), .b(new_n282), .o1(new_n283));
  xorc02aa1n12x5               g188(.a(\a[25] ), .b(\b[24] ), .out0(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  nona22aa1n02x4               g190(.a(new_n285), .b(new_n278), .c(new_n279), .out0(new_n286));
  aoib12aa1n02x5               g191(.a(new_n286), .b(new_n234), .c(new_n274), .out0(new_n287));
  aoi022aa1n02x5               g192(.a(new_n283), .b(new_n284), .c(new_n276), .d(new_n287), .o1(\s[25] ));
  aoai13aa1n02x7               g193(.a(new_n284), .b(new_n281), .c(new_n199), .d(new_n275), .o1(new_n289));
  orn002aa1n02x5               g194(.a(\a[25] ), .b(\b[24] ), .o(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n285), .c(new_n276), .d(new_n282), .o1(new_n291));
  tech160nm_fixorc02aa1n02p5x5 g196(.a(\a[26] ), .b(\b[25] ), .out0(new_n292));
  norb02aa1n02x5               g197(.a(new_n290), .b(new_n292), .out0(new_n293));
  aoi022aa1n03x5               g198(.a(new_n291), .b(new_n292), .c(new_n289), .d(new_n293), .o1(\s[26] ));
  and002aa1n12x5               g199(.a(new_n292), .b(new_n284), .o(new_n295));
  nano32aa1n03x7               g200(.a(new_n274), .b(new_n295), .c(new_n253), .d(new_n213), .out0(new_n296));
  aoai13aa1n12x5               g201(.a(new_n296), .b(new_n208), .c(new_n128), .d(new_n201), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n290), .carry(new_n298));
  aobi12aa1n06x5               g203(.a(new_n298), .b(new_n281), .c(new_n295), .out0(new_n299));
  nanp02aa1n02x5               g204(.a(new_n297), .b(new_n299), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  inv000aa1d42x5               g206(.a(new_n301), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n298), .b(new_n302), .o1(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(new_n281), .c(new_n295), .o1(new_n304));
  aoi022aa1n02x5               g209(.a(new_n300), .b(new_n301), .c(new_n297), .d(new_n304), .o1(\s[27] ));
  aob012aa1n06x5               g210(.a(new_n298), .b(new_n281), .c(new_n295), .out0(new_n306));
  aoai13aa1n02x7               g211(.a(new_n301), .b(new_n306), .c(new_n199), .d(new_n296), .o1(new_n307));
  nor042aa1d18x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  inv040aa1n06x5               g213(.a(new_n308), .o1(new_n309));
  aoai13aa1n03x5               g214(.a(new_n309), .b(new_n302), .c(new_n297), .d(new_n299), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n308), .o1(new_n312));
  aoi022aa1n03x5               g217(.a(new_n310), .b(new_n311), .c(new_n307), .d(new_n312), .o1(\s[28] ));
  and002aa1n02x5               g218(.a(new_n311), .b(new_n301), .o(new_n314));
  aoai13aa1n03x5               g219(.a(new_n314), .b(new_n306), .c(new_n199), .d(new_n296), .o1(new_n315));
  inv000aa1d42x5               g220(.a(new_n314), .o1(new_n316));
  oao003aa1n03x5               g221(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n316), .c(new_n297), .d(new_n299), .o1(new_n318));
  xorc02aa1n02x5               g223(.a(\a[29] ), .b(\b[28] ), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n317), .b(new_n319), .out0(new_n320));
  aoi022aa1n03x5               g225(.a(new_n318), .b(new_n319), .c(new_n315), .d(new_n320), .o1(\s[29] ));
  xorb03aa1n02x5               g226(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1d15x5               g227(.a(new_n302), .b(new_n311), .c(new_n319), .out0(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n306), .c(new_n199), .d(new_n296), .o1(new_n324));
  inv000aa1d42x5               g229(.a(new_n323), .o1(new_n325));
  tech160nm_fioaoi03aa1n03p5x5 g230(.a(\a[29] ), .b(\b[28] ), .c(new_n317), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n326), .o1(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n325), .c(new_n297), .d(new_n299), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[30] ), .b(\b[29] ), .out0(new_n329));
  and002aa1n02x5               g234(.a(\b[28] ), .b(\a[29] ), .o(new_n330));
  oabi12aa1n02x5               g235(.a(new_n329), .b(\a[29] ), .c(\b[28] ), .out0(new_n331));
  oab012aa1n02x4               g236(.a(new_n331), .b(new_n317), .c(new_n330), .out0(new_n332));
  aoi022aa1n03x5               g237(.a(new_n328), .b(new_n329), .c(new_n324), .d(new_n332), .o1(\s[30] ));
  nano32aa1n02x4               g238(.a(new_n302), .b(new_n329), .c(new_n311), .d(new_n319), .out0(new_n334));
  aoai13aa1n02x7               g239(.a(new_n334), .b(new_n306), .c(new_n199), .d(new_n296), .o1(new_n335));
  xorc02aa1n02x5               g240(.a(\a[31] ), .b(\b[30] ), .out0(new_n336));
  inv000aa1d42x5               g241(.a(\a[30] ), .o1(new_n337));
  inv000aa1d42x5               g242(.a(\b[29] ), .o1(new_n338));
  oabi12aa1n02x5               g243(.a(new_n336), .b(\a[30] ), .c(\b[29] ), .out0(new_n339));
  oaoi13aa1n02x5               g244(.a(new_n339), .b(new_n326), .c(new_n337), .d(new_n338), .o1(new_n340));
  inv000aa1n02x5               g245(.a(new_n334), .o1(new_n341));
  oaoi03aa1n02x5               g246(.a(new_n337), .b(new_n338), .c(new_n326), .o1(new_n342));
  aoai13aa1n03x5               g247(.a(new_n342), .b(new_n341), .c(new_n297), .d(new_n299), .o1(new_n343));
  aoi022aa1n03x5               g248(.a(new_n343), .b(new_n336), .c(new_n335), .d(new_n340), .o1(\s[31] ));
  aoi112aa1n02x5               g249(.a(new_n109), .b(new_n99), .c(new_n101), .d(new_n102), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n103), .c(new_n109), .o1(\s[3] ));
  aoi112aa1n02x5               g251(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n108), .o1(new_n347));
  aoib12aa1n02x5               g252(.a(new_n347), .b(new_n157), .c(new_n104), .out0(\s[4] ));
  norb02aa1n02x5               g253(.a(new_n120), .b(new_n119), .out0(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n349), .b(new_n110), .c(new_n111), .out0(\s[5] ));
  norb02aa1n02x5               g255(.a(new_n118), .b(new_n117), .out0(new_n351));
  inv000aa1d42x5               g256(.a(new_n119), .o1(new_n352));
  nanp02aa1n02x5               g257(.a(new_n157), .b(new_n349), .o1(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n351), .b(new_n353), .c(new_n352), .out0(\s[6] ));
  nona22aa1n02x4               g259(.a(new_n353), .b(new_n119), .c(new_n117), .out0(new_n355));
  aoai13aa1n02x5               g260(.a(new_n118), .b(new_n124), .c(new_n157), .d(new_n349), .o1(new_n356));
  nano22aa1n02x4               g261(.a(new_n114), .b(new_n115), .c(new_n118), .out0(new_n357));
  aboi22aa1n03x5               g262(.a(new_n123), .b(new_n356), .c(new_n355), .d(new_n357), .out0(\s[7] ));
  norb02aa1n02x5               g263(.a(new_n113), .b(new_n112), .out0(new_n359));
  orn002aa1n02x5               g264(.a(\a[7] ), .b(\b[6] ), .o(new_n360));
  aoai13aa1n02x5               g265(.a(new_n357), .b(new_n124), .c(new_n157), .d(new_n349), .o1(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n359), .b(new_n361), .c(new_n360), .out0(\s[8] ));
  aoi113aa1n02x5               g267(.a(new_n126), .b(new_n129), .c(new_n123), .d(new_n124), .e(new_n125), .o1(new_n363));
  aoi022aa1n02x5               g268(.a(new_n128), .b(new_n129), .c(new_n158), .d(new_n363), .o1(\s[9] ));
endmodule


