// Benchmark "adder" written by ABC on Thu Jul 18 04:38:34 2024

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
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n257, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n264, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n325, new_n326, new_n327, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n336, new_n337, new_n338,
    new_n339, new_n340, new_n342, new_n344, new_n345, new_n347, new_n348,
    new_n349, new_n350, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  nor002aa1n06x5               g002(.a(\b[2] ), .b(\a[3] ), .o1(new_n98));
  nanp02aa1n09x5               g003(.a(\b[2] ), .b(\a[3] ), .o1(new_n99));
  norb02aa1n02x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  nor042aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  aoi022aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .c(\a[1] ), .d(\b[0] ), .o1(new_n102));
  oai022aa1n04x5               g007(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  oaoi13aa1n04x5               g008(.a(new_n103), .b(new_n100), .c(new_n102), .d(new_n101), .o1(new_n104));
  nand42aa1n08x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  tech160nm_fioai012aa1n04x5   g010(.a(new_n105), .b(\b[5] ), .c(\a[6] ), .o1(new_n106));
  nand42aa1n06x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  norp02aa1n02x5               g012(.a(\b[7] ), .b(\a[8] ), .o1(new_n108));
  tech160nm_finand02aa1n05x5   g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nano23aa1n02x5               g014(.a(new_n106), .b(new_n108), .c(new_n109), .d(new_n107), .out0(new_n110));
  nor002aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nand42aa1n08x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor042aa1n03x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n02x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  nand42aa1n02x5               g020(.a(new_n110), .b(new_n115), .o1(new_n116));
  nanb03aa1n02x5               g021(.a(new_n113), .b(new_n114), .c(new_n109), .out0(new_n117));
  oai022aa1n02x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  oai112aa1n02x5               g023(.a(new_n118), .b(new_n107), .c(\b[7] ), .d(\a[8] ), .o1(new_n119));
  tech160nm_fiao0012aa1n02p5x5 g024(.a(new_n108), .b(new_n113), .c(new_n109), .o(new_n120));
  oab012aa1n09x5               g025(.a(new_n120), .b(new_n119), .c(new_n117), .out0(new_n121));
  oai012aa1n09x5               g026(.a(new_n121), .b(new_n116), .c(new_n104), .o1(new_n122));
  nand02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n97), .b(new_n122), .c(new_n123), .o1(new_n124));
  xnrb03aa1n02x5               g029(.a(new_n124), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nand22aa1n04x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  oai022aa1d24x5               g031(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n127));
  nor022aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nano23aa1n06x5               g033(.a(new_n97), .b(new_n128), .c(new_n126), .d(new_n123), .out0(new_n129));
  ao0022aa1n03x7               g034(.a(new_n122), .b(new_n129), .c(new_n127), .d(new_n126), .o(new_n130));
  xorb03aa1n02x5               g035(.a(new_n130), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  inv040aa1d32x5               g036(.a(\a[12] ), .o1(new_n132));
  inv000aa1d48x5               g037(.a(\b[11] ), .o1(new_n133));
  nand02aa1d10x5               g038(.a(new_n133), .b(new_n132), .o1(new_n134));
  nand42aa1n06x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\a[11] ), .o1(new_n136));
  inv000aa1d42x5               g041(.a(\b[10] ), .o1(new_n137));
  tech160nm_fioaoi03aa1n03p5x5 g042(.a(new_n136), .b(new_n137), .c(new_n130), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n134), .c(new_n135), .out0(\s[12] ));
  nanb02aa1n06x5               g044(.a(new_n98), .b(new_n99), .out0(new_n140));
  inv030aa1n02x5               g045(.a(new_n101), .o1(new_n141));
  nand42aa1n04x5               g046(.a(\b[0] ), .b(\a[1] ), .o1(new_n142));
  aob012aa1n06x5               g047(.a(new_n142), .b(\b[1] ), .c(\a[2] ), .out0(new_n143));
  inv030aa1n02x5               g048(.a(new_n103), .o1(new_n144));
  aoai13aa1n06x5               g049(.a(new_n144), .b(new_n140), .c(new_n143), .d(new_n141), .o1(new_n145));
  nanp03aa1n03x5               g050(.a(new_n145), .b(new_n110), .c(new_n115), .o1(new_n146));
  xnrc02aa1n02x5               g051(.a(\b[10] ), .b(\a[11] ), .out0(new_n147));
  nanp02aa1n02x5               g052(.a(new_n134), .b(new_n135), .o1(new_n148));
  nona22aa1n06x5               g053(.a(new_n129), .b(new_n147), .c(new_n148), .out0(new_n149));
  oai112aa1n06x5               g054(.a(new_n134), .b(new_n135), .c(new_n137), .d(new_n136), .o1(new_n150));
  oai112aa1n06x5               g055(.a(new_n127), .b(new_n126), .c(\b[10] ), .d(\a[11] ), .o1(new_n151));
  aoi112aa1n03x5               g056(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n152));
  norb02aa1n09x5               g057(.a(new_n134), .b(new_n152), .out0(new_n153));
  oai012aa1d24x5               g058(.a(new_n153), .b(new_n151), .c(new_n150), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  aoai13aa1n02x5               g060(.a(new_n155), .b(new_n149), .c(new_n146), .d(new_n121), .o1(new_n156));
  nor002aa1d32x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand02aa1n06x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  norb02aa1n02x5               g063(.a(new_n158), .b(new_n157), .out0(new_n159));
  inv000aa1n02x5               g064(.a(new_n149), .o1(new_n160));
  aoi112aa1n02x5               g065(.a(new_n159), .b(new_n154), .c(new_n122), .d(new_n160), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n161), .b(new_n156), .c(new_n159), .o1(\s[13] ));
  aoi012aa1n02x5               g067(.a(new_n157), .b(new_n156), .c(new_n158), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor022aa1n16x5               g069(.a(\b[13] ), .b(\a[14] ), .o1(new_n165));
  nanp02aa1n06x5               g070(.a(\b[13] ), .b(\a[14] ), .o1(new_n166));
  nona23aa1d24x5               g071(.a(new_n166), .b(new_n158), .c(new_n157), .d(new_n165), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n06x5               g073(.a(new_n168), .b(new_n154), .c(new_n122), .d(new_n160), .o1(new_n169));
  oa0012aa1n02x5               g074(.a(new_n166), .b(new_n165), .c(new_n157), .o(new_n170));
  inv000aa1n02x5               g075(.a(new_n170), .o1(new_n171));
  nor042aa1n02x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand22aa1n04x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n169), .c(new_n171), .out0(\s[15] ));
  nanp02aa1n03x5               g080(.a(new_n169), .b(new_n171), .o1(new_n176));
  nor042aa1n09x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand02aa1n06x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  nanb02aa1n02x5               g083(.a(new_n177), .b(new_n178), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n172), .c(new_n176), .d(new_n173), .o1(new_n180));
  aoi112aa1n02x7               g085(.a(new_n172), .b(new_n179), .c(new_n176), .d(new_n173), .o1(new_n181));
  nanb02aa1n03x5               g086(.a(new_n181), .b(new_n180), .out0(\s[16] ));
  nona23aa1n02x4               g087(.a(new_n126), .b(new_n123), .c(new_n97), .d(new_n128), .out0(new_n183));
  nona23aa1n09x5               g088(.a(new_n178), .b(new_n173), .c(new_n172), .d(new_n177), .out0(new_n184));
  nor042aa1n09x5               g089(.a(new_n184), .b(new_n167), .o1(new_n185));
  nona32aa1n02x4               g090(.a(new_n185), .b(new_n183), .c(new_n148), .d(new_n147), .out0(new_n186));
  inv000aa1n02x5               g091(.a(new_n172), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n177), .o1(new_n188));
  inv000aa1d42x5               g093(.a(new_n178), .o1(new_n189));
  oai022aa1n02x5               g094(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n190));
  nanp03aa1n02x5               g095(.a(new_n190), .b(new_n166), .c(new_n173), .o1(new_n191));
  aoai13aa1n03x5               g096(.a(new_n188), .b(new_n189), .c(new_n191), .d(new_n187), .o1(new_n192));
  aoi012aa1n09x5               g097(.a(new_n192), .b(new_n154), .c(new_n185), .o1(new_n193));
  aoai13aa1n12x5               g098(.a(new_n193), .b(new_n186), .c(new_n146), .d(new_n121), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g100(.a(\a[17] ), .o1(new_n196));
  nanb02aa1n02x5               g101(.a(\b[16] ), .b(new_n196), .out0(new_n197));
  oaib12aa1n06x5               g102(.a(new_n194), .b(new_n196), .c(\b[16] ), .out0(new_n198));
  xorc02aa1n02x5               g103(.a(\a[18] ), .b(\b[17] ), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n198), .c(new_n197), .out0(\s[18] ));
  norb02aa1n09x5               g105(.a(new_n185), .b(new_n149), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n154), .b(new_n185), .o1(new_n202));
  inv000aa1d42x5               g107(.a(\a[16] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(\b[15] ), .o1(new_n204));
  nanp02aa1n02x5               g109(.a(new_n191), .b(new_n187), .o1(new_n205));
  oaoi03aa1n02x5               g110(.a(new_n203), .b(new_n204), .c(new_n205), .o1(new_n206));
  nand42aa1n03x5               g111(.a(new_n202), .b(new_n206), .o1(new_n207));
  inv040aa1d32x5               g112(.a(\a[18] ), .o1(new_n208));
  xroi22aa1d06x4               g113(.a(new_n196), .b(\b[16] ), .c(new_n208), .d(\b[17] ), .out0(new_n209));
  aoai13aa1n02x5               g114(.a(new_n209), .b(new_n207), .c(new_n122), .d(new_n201), .o1(new_n210));
  tech160nm_fioaoi03aa1n03p5x5 g115(.a(\a[18] ), .b(\b[17] ), .c(new_n197), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n211), .o1(new_n212));
  nor042aa1n03x5               g117(.a(\b[18] ), .b(\a[19] ), .o1(new_n213));
  nand02aa1n06x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  norb02aa1n03x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n210), .c(new_n212), .out0(\s[19] ));
  xnrc02aa1n02x5               g121(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nand42aa1n02x5               g122(.a(new_n210), .b(new_n212), .o1(new_n218));
  nor002aa1n03x5               g123(.a(\b[19] ), .b(\a[20] ), .o1(new_n219));
  nand02aa1n03x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nanb02aa1n02x5               g125(.a(new_n219), .b(new_n220), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n213), .c(new_n218), .d(new_n214), .o1(new_n222));
  aoai13aa1n02x5               g127(.a(new_n215), .b(new_n211), .c(new_n194), .d(new_n209), .o1(new_n223));
  nona22aa1n02x4               g128(.a(new_n223), .b(new_n221), .c(new_n213), .out0(new_n224));
  nanp02aa1n02x5               g129(.a(new_n222), .b(new_n224), .o1(\s[20] ));
  nanb03aa1n09x5               g130(.a(new_n221), .b(new_n209), .c(new_n215), .out0(new_n226));
  nanb02aa1n02x5               g131(.a(new_n226), .b(new_n194), .out0(new_n227));
  aoi012aa1n06x5               g132(.a(new_n207), .b(new_n122), .c(new_n201), .o1(new_n228));
  nanb03aa1n02x5               g133(.a(new_n219), .b(new_n220), .c(new_n214), .out0(new_n229));
  inv000aa1d42x5               g134(.a(\b[17] ), .o1(new_n230));
  oaih22aa1n04x5               g135(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n231));
  oai122aa1n02x7               g136(.a(new_n231), .b(\a[19] ), .c(\b[18] ), .d(new_n208), .e(new_n230), .o1(new_n232));
  tech160nm_fiao0012aa1n04x5   g137(.a(new_n219), .b(new_n213), .c(new_n220), .o(new_n233));
  oabi12aa1n06x5               g138(.a(new_n233), .b(new_n232), .c(new_n229), .out0(new_n234));
  oabi12aa1n06x5               g139(.a(new_n234), .b(new_n228), .c(new_n226), .out0(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  nano22aa1n03x5               g142(.a(new_n219), .b(new_n214), .c(new_n220), .out0(new_n238));
  oai022aa1n02x5               g143(.a(new_n208), .b(new_n230), .c(\b[18] ), .d(\a[19] ), .o1(new_n239));
  norb02aa1n02x5               g144(.a(new_n231), .b(new_n239), .out0(new_n240));
  aoi112aa1n02x5               g145(.a(new_n233), .b(new_n237), .c(new_n240), .d(new_n238), .o1(new_n241));
  aoi022aa1n02x5               g146(.a(new_n235), .b(new_n237), .c(new_n227), .d(new_n241), .o1(\s[21] ));
  nor042aa1n04x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  tech160nm_fixnrc02aa1n05x5   g148(.a(\b[21] ), .b(\a[22] ), .out0(new_n244));
  aoai13aa1n03x5               g149(.a(new_n244), .b(new_n243), .c(new_n235), .d(new_n237), .o1(new_n245));
  nanp02aa1n02x5               g150(.a(new_n235), .b(new_n237), .o1(new_n246));
  nona22aa1n02x4               g151(.a(new_n246), .b(new_n244), .c(new_n243), .out0(new_n247));
  nanp02aa1n02x5               g152(.a(new_n247), .b(new_n245), .o1(\s[22] ));
  nor042aa1n09x5               g153(.a(new_n244), .b(new_n236), .o1(new_n249));
  nano32aa1n02x4               g154(.a(new_n221), .b(new_n209), .c(new_n249), .d(new_n215), .out0(new_n250));
  inv000aa1d42x5               g155(.a(\a[22] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\b[21] ), .o1(new_n252));
  oao003aa1n12x5               g157(.a(new_n251), .b(new_n252), .c(new_n243), .carry(new_n253));
  aoi012aa1n02x5               g158(.a(new_n253), .b(new_n234), .c(new_n249), .o1(new_n254));
  oaib12aa1n06x5               g159(.a(new_n254), .b(new_n228), .c(new_n250), .out0(new_n255));
  xorb03aa1n02x5               g160(.a(new_n255), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  xorc02aa1n12x5               g162(.a(\a[23] ), .b(\b[22] ), .out0(new_n258));
  xnrc02aa1n12x5               g163(.a(\b[23] ), .b(\a[24] ), .out0(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n257), .c(new_n255), .d(new_n258), .o1(new_n260));
  nanp02aa1n02x5               g165(.a(new_n255), .b(new_n258), .o1(new_n261));
  nona22aa1n02x4               g166(.a(new_n261), .b(new_n259), .c(new_n257), .out0(new_n262));
  nanp02aa1n02x5               g167(.a(new_n262), .b(new_n260), .o1(\s[24] ));
  inv000aa1d42x5               g168(.a(new_n249), .o1(new_n264));
  norb02aa1n03x5               g169(.a(new_n258), .b(new_n259), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  nona32aa1n03x5               g171(.a(new_n194), .b(new_n266), .c(new_n264), .d(new_n226), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n249), .b(new_n233), .c(new_n240), .d(new_n238), .o1(new_n268));
  inv000aa1n02x5               g173(.a(new_n253), .o1(new_n269));
  aoi112aa1n02x5               g174(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n270));
  oab012aa1n02x4               g175(.a(new_n270), .b(\a[24] ), .c(\b[23] ), .out0(new_n271));
  aoai13aa1n12x5               g176(.a(new_n271), .b(new_n266), .c(new_n268), .d(new_n269), .o1(new_n272));
  inv000aa1n02x5               g177(.a(new_n272), .o1(new_n273));
  nanp02aa1n03x5               g178(.a(new_n267), .b(new_n273), .o1(new_n274));
  tech160nm_fixorc02aa1n03p5x5 g179(.a(\a[25] ), .b(\b[24] ), .out0(new_n275));
  aoai13aa1n06x5               g180(.a(new_n265), .b(new_n253), .c(new_n234), .d(new_n249), .o1(new_n276));
  nano22aa1n02x4               g181(.a(new_n275), .b(new_n276), .c(new_n271), .out0(new_n277));
  aoi022aa1n02x5               g182(.a(new_n274), .b(new_n275), .c(new_n267), .d(new_n277), .o1(\s[25] ));
  norp02aa1n02x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[25] ), .b(\a[26] ), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n280), .b(new_n279), .c(new_n274), .d(new_n275), .o1(new_n281));
  nano22aa1n02x4               g186(.a(new_n228), .b(new_n250), .c(new_n265), .out0(new_n282));
  oai012aa1n02x7               g187(.a(new_n275), .b(new_n282), .c(new_n272), .o1(new_n283));
  nona22aa1n02x4               g188(.a(new_n283), .b(new_n280), .c(new_n279), .out0(new_n284));
  nanp02aa1n03x5               g189(.a(new_n281), .b(new_n284), .o1(\s[26] ));
  norp02aa1n02x5               g190(.a(\b[26] ), .b(\a[27] ), .o1(new_n286));
  and002aa1n02x5               g191(.a(\b[26] ), .b(\a[27] ), .o(new_n287));
  norp02aa1n02x5               g192(.a(new_n287), .b(new_n286), .o1(new_n288));
  nona23aa1n02x4               g193(.a(new_n275), .b(new_n258), .c(new_n280), .d(new_n259), .out0(new_n289));
  nona32aa1d24x5               g194(.a(new_n194), .b(new_n289), .c(new_n264), .d(new_n226), .out0(new_n290));
  nanp02aa1n02x5               g195(.a(\b[25] ), .b(\a[26] ), .o1(new_n291));
  norb02aa1n02x5               g196(.a(new_n275), .b(new_n280), .out0(new_n292));
  oai022aa1n02x5               g197(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n293));
  aoi022aa1n12x5               g198(.a(new_n272), .b(new_n292), .c(new_n291), .d(new_n293), .o1(new_n294));
  xnbna2aa1n03x5               g199(.a(new_n288), .b(new_n294), .c(new_n290), .out0(\s[27] ));
  inv000aa1d42x5               g200(.a(\a[27] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\b[26] ), .o1(new_n297));
  inv000aa1n02x5               g202(.a(new_n292), .o1(new_n298));
  nanp02aa1n02x5               g203(.a(new_n293), .b(new_n291), .o1(new_n299));
  aoai13aa1n12x5               g204(.a(new_n299), .b(new_n298), .c(new_n276), .d(new_n271), .o1(new_n300));
  obai22aa1n02x5               g205(.a(new_n290), .b(new_n300), .c(new_n296), .d(new_n297), .out0(new_n301));
  inv000aa1n03x5               g206(.a(new_n286), .o1(new_n302));
  aoai13aa1n03x5               g207(.a(new_n302), .b(new_n287), .c(new_n294), .d(new_n290), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n304), .b(new_n286), .o1(new_n305));
  aoi022aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n301), .d(new_n305), .o1(\s[28] ));
  inv000aa1d42x5               g211(.a(\a[28] ), .o1(new_n307));
  xroi22aa1d04x5               g212(.a(new_n296), .b(\b[26] ), .c(new_n307), .d(\b[27] ), .out0(new_n308));
  oaib12aa1n03x5               g213(.a(new_n308), .b(new_n300), .c(new_n290), .out0(new_n309));
  inv000aa1n03x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n302), .carry(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n310), .c(new_n294), .d(new_n290), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n142), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g221(.a(new_n304), .b(new_n313), .c(new_n288), .o(new_n317));
  oaib12aa1n03x5               g222(.a(new_n317), .b(new_n300), .c(new_n290), .out0(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  oao003aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .c(new_n311), .carry(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n319), .c(new_n294), .d(new_n290), .o1(new_n321));
  xorc02aa1n02x5               g226(.a(\a[30] ), .b(\b[29] ), .out0(new_n322));
  norb02aa1n02x5               g227(.a(new_n320), .b(new_n322), .out0(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n318), .d(new_n323), .o1(\s[30] ));
  nano22aa1n02x4               g229(.a(new_n310), .b(new_n313), .c(new_n322), .out0(new_n325));
  oaib12aa1n03x5               g230(.a(new_n325), .b(new_n300), .c(new_n290), .out0(new_n326));
  xorc02aa1n02x5               g231(.a(\a[31] ), .b(\b[30] ), .out0(new_n327));
  and002aa1n02x5               g232(.a(\b[29] ), .b(\a[30] ), .o(new_n328));
  oabi12aa1n02x5               g233(.a(new_n327), .b(\a[30] ), .c(\b[29] ), .out0(new_n329));
  oab012aa1n02x4               g234(.a(new_n329), .b(new_n320), .c(new_n328), .out0(new_n330));
  inv000aa1n02x5               g235(.a(new_n325), .o1(new_n331));
  oao003aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .c(new_n320), .carry(new_n332));
  aoai13aa1n03x5               g237(.a(new_n332), .b(new_n331), .c(new_n294), .d(new_n290), .o1(new_n333));
  aoi022aa1n03x5               g238(.a(new_n333), .b(new_n327), .c(new_n326), .d(new_n330), .o1(\s[31] ));
  xnbna2aa1n03x5               g239(.a(new_n100), .b(new_n143), .c(new_n141), .out0(\s[3] ));
  inv000aa1d42x5               g240(.a(\a[4] ), .o1(new_n336));
  inv000aa1d42x5               g241(.a(\b[3] ), .o1(new_n337));
  nanp02aa1n02x5               g242(.a(new_n337), .b(new_n336), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n140), .b(new_n143), .c(new_n141), .o1(new_n339));
  aoi112aa1n02x5               g244(.a(new_n339), .b(new_n98), .c(new_n338), .d(new_n112), .o1(new_n340));
  aoi013aa1n02x4               g245(.a(new_n340), .b(new_n145), .c(new_n112), .d(new_n338), .o1(\s[4] ));
  norb02aa1n02x5               g246(.a(new_n105), .b(new_n111), .out0(new_n342));
  xobna2aa1n03x5               g247(.a(new_n342), .b(new_n145), .c(new_n112), .out0(\s[5] ));
  aoai13aa1n03x5               g248(.a(new_n105), .b(new_n111), .c(new_n145), .d(new_n112), .o1(new_n344));
  xorc02aa1n02x5               g249(.a(\a[6] ), .b(\b[5] ), .out0(new_n345));
  xnrc02aa1n02x5               g250(.a(new_n344), .b(new_n345), .out0(\s[6] ));
  norb02aa1n02x5               g251(.a(new_n114), .b(new_n113), .out0(new_n347));
  oaoi03aa1n02x5               g252(.a(\a[6] ), .b(\b[5] ), .c(new_n344), .o1(new_n348));
  oai012aa1n02x5               g253(.a(new_n107), .b(\b[6] ), .c(\a[7] ), .o1(new_n349));
  aoi122aa1n06x5               g254(.a(new_n349), .b(\b[6] ), .c(\a[7] ), .d(new_n344), .e(new_n345), .o1(new_n350));
  oab012aa1n02x4               g255(.a(new_n350), .b(new_n348), .c(new_n347), .out0(\s[7] ));
  norp02aa1n03x5               g256(.a(new_n350), .b(new_n113), .o1(new_n352));
  xnrb03aa1n03x5               g257(.a(new_n352), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g258(.a(new_n122), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


