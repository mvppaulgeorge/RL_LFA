// Benchmark "adder" written by ABC on Thu Jul 18 06:39:07 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n174, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n232, new_n233, new_n234, new_n235,
    new_n236, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n284, new_n285, new_n286, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n345,
    new_n348, new_n350, new_n352;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nanp02aa1n04x5               g002(.a(\b[5] ), .b(\a[6] ), .o1(new_n98));
  norp02aa1n04x5               g003(.a(\b[5] ), .b(\a[6] ), .o1(new_n99));
  nor002aa1n02x5               g004(.a(\b[4] ), .b(\a[5] ), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[4] ), .b(\a[5] ), .o1(new_n101));
  nona23aa1n03x5               g006(.a(new_n98), .b(new_n101), .c(new_n100), .d(new_n99), .out0(new_n102));
  nor002aa1d32x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[7] ), .b(\a[8] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  nor002aa1n03x5               g010(.a(\b[6] ), .b(\a[7] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n105), .b(new_n104), .c(new_n106), .d(new_n103), .out0(new_n107));
  nor002aa1n03x5               g012(.a(new_n107), .b(new_n102), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[1] ), .b(\a[2] ), .o1(new_n109));
  nand22aa1n03x5               g014(.a(\b[0] ), .b(\a[1] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(\b[1] ), .b(\a[2] ), .o1(new_n111));
  aoi012aa1n06x5               g016(.a(new_n109), .b(new_n110), .c(new_n111), .o1(new_n112));
  nand42aa1n04x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[3] ), .b(\a[4] ), .o1(new_n114));
  nor022aa1n04x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nand42aa1n03x5               g020(.a(\b[2] ), .b(\a[3] ), .o1(new_n116));
  nona23aa1n02x4               g021(.a(new_n113), .b(new_n116), .c(new_n115), .d(new_n114), .out0(new_n117));
  tech160nm_fiao0012aa1n02p5x5 g022(.a(new_n114), .b(new_n115), .c(new_n113), .o(new_n118));
  oabi12aa1n06x5               g023(.a(new_n118), .b(new_n117), .c(new_n112), .out0(new_n119));
  inv000aa1d42x5               g024(.a(new_n103), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(new_n106), .b(new_n104), .o1(new_n121));
  tech160nm_fioai012aa1n04x5   g026(.a(new_n98), .b(new_n100), .c(new_n99), .o1(new_n122));
  oai112aa1n03x5               g027(.a(new_n120), .b(new_n121), .c(new_n107), .d(new_n122), .o1(new_n123));
  nor042aa1n06x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nand42aa1n08x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  aoai13aa1n03x5               g031(.a(new_n126), .b(new_n123), .c(new_n119), .d(new_n108), .o1(new_n127));
  nor042aa1n04x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1n16x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  norb02aa1n02x5               g034(.a(new_n129), .b(new_n128), .out0(new_n130));
  xnbna2aa1n03x5               g035(.a(new_n130), .b(new_n127), .c(new_n97), .out0(\s[10] ));
  nor002aa1n04x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n06x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n09x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  inv000aa1n02x5               g039(.a(new_n134), .o1(new_n135));
  nona22aa1n02x5               g040(.a(new_n127), .b(new_n128), .c(new_n124), .out0(new_n136));
  xnbna2aa1n03x5               g041(.a(new_n135), .b(new_n136), .c(new_n129), .out0(\s[11] ));
  nanp03aa1n02x5               g042(.a(new_n136), .b(new_n129), .c(new_n134), .o1(new_n138));
  norp02aa1n02x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  tech160nm_finand02aa1n05x5   g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n03x4               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  inv000aa1n02x5               g046(.a(new_n141), .o1(new_n142));
  oaoi13aa1n02x5               g047(.a(new_n142), .b(new_n138), .c(\a[11] ), .d(\b[10] ), .o1(new_n143));
  aoi113aa1n02x5               g048(.a(new_n132), .b(new_n141), .c(new_n136), .d(new_n134), .e(new_n129), .o1(new_n144));
  norp02aa1n03x5               g049(.a(new_n143), .b(new_n144), .o1(\s[12] ));
  inv000aa1n02x5               g050(.a(new_n112), .o1(new_n146));
  nano23aa1n02x5               g051(.a(new_n115), .b(new_n114), .c(new_n116), .d(new_n113), .out0(new_n147));
  aoai13aa1n04x5               g052(.a(new_n108), .b(new_n118), .c(new_n146), .d(new_n147), .o1(new_n148));
  nanb02aa1n02x5               g053(.a(new_n103), .b(new_n104), .out0(new_n149));
  nanb02aa1n02x5               g054(.a(new_n106), .b(new_n105), .out0(new_n150));
  nor043aa1n02x5               g055(.a(new_n122), .b(new_n150), .c(new_n149), .o1(new_n151));
  nano22aa1n03x7               g056(.a(new_n151), .b(new_n120), .c(new_n121), .out0(new_n152));
  nano23aa1n03x5               g057(.a(new_n124), .b(new_n128), .c(new_n129), .d(new_n125), .out0(new_n153));
  nand03aa1n02x5               g058(.a(new_n153), .b(new_n134), .c(new_n141), .o1(new_n154));
  oai112aa1n02x5               g059(.a(new_n129), .b(new_n133), .c(new_n128), .d(new_n124), .o1(new_n155));
  nona22aa1n06x5               g060(.a(new_n155), .b(new_n139), .c(new_n132), .out0(new_n156));
  nanp02aa1n02x5               g061(.a(new_n156), .b(new_n140), .o1(new_n157));
  aoai13aa1n06x5               g062(.a(new_n157), .b(new_n154), .c(new_n148), .d(new_n152), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv000aa1d42x5               g064(.a(\a[13] ), .o1(new_n160));
  inv000aa1d42x5               g065(.a(\b[12] ), .o1(new_n161));
  oaoi03aa1n02x5               g066(.a(new_n160), .b(new_n161), .c(new_n158), .o1(new_n162));
  xnrb03aa1n03x5               g067(.a(new_n162), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  inv000aa1n02x5               g068(.a(new_n154), .o1(new_n164));
  aoai13aa1n03x5               g069(.a(new_n164), .b(new_n123), .c(new_n119), .d(new_n108), .o1(new_n165));
  norp02aa1n02x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nand42aa1n03x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  norp02aa1n04x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nona23aa1n09x5               g074(.a(new_n169), .b(new_n167), .c(new_n166), .d(new_n168), .out0(new_n170));
  aoai13aa1n06x5               g075(.a(new_n169), .b(new_n168), .c(new_n160), .d(new_n161), .o1(new_n171));
  aoai13aa1n03x5               g076(.a(new_n171), .b(new_n170), .c(new_n165), .d(new_n157), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor002aa1n12x5               g078(.a(\b[14] ), .b(\a[15] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(new_n174), .o1(new_n175));
  nano23aa1n02x4               g080(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n176));
  inv000aa1d42x5               g081(.a(new_n171), .o1(new_n177));
  nand42aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norb02aa1n06x4               g083(.a(new_n178), .b(new_n174), .out0(new_n179));
  aoai13aa1n02x5               g084(.a(new_n179), .b(new_n177), .c(new_n158), .d(new_n176), .o1(new_n180));
  nor002aa1n04x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  tech160nm_finand02aa1n05x5   g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n12x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  tech160nm_fiaoi012aa1n03p5x5 g089(.a(new_n184), .b(new_n180), .c(new_n175), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n174), .b(new_n183), .c(new_n172), .d(new_n178), .o1(new_n186));
  norp02aa1n03x5               g091(.a(new_n185), .b(new_n186), .o1(\s[16] ));
  nano32aa1n03x7               g092(.a(new_n154), .b(new_n183), .c(new_n176), .d(new_n179), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n188), .b(new_n123), .c(new_n108), .d(new_n119), .o1(new_n189));
  nano22aa1n09x5               g094(.a(new_n170), .b(new_n179), .c(new_n183), .out0(new_n190));
  oai022aa1n02x5               g095(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n191));
  aoai13aa1n03x5               g096(.a(new_n178), .b(new_n174), .c(new_n191), .d(new_n169), .o1(new_n192));
  oaoi03aa1n02x5               g097(.a(\a[16] ), .b(\b[15] ), .c(new_n192), .o1(new_n193));
  aoi013aa1n09x5               g098(.a(new_n193), .b(new_n190), .c(new_n156), .d(new_n140), .o1(new_n194));
  xnrc02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .out0(new_n195));
  xobna2aa1n03x5               g100(.a(new_n195), .b(new_n189), .c(new_n194), .out0(\s[17] ));
  nor042aa1n06x5               g101(.a(\b[16] ), .b(\a[17] ), .o1(new_n197));
  inv000aa1n02x5               g102(.a(new_n197), .o1(new_n198));
  aoai13aa1n03x5               g103(.a(new_n198), .b(new_n195), .c(new_n189), .d(new_n194), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g105(.a(\a[17] ), .o1(new_n201));
  inv020aa1n04x5               g106(.a(\a[18] ), .o1(new_n202));
  xroi22aa1d06x4               g107(.a(new_n201), .b(\b[16] ), .c(new_n202), .d(\b[17] ), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  oaoi03aa1n02x5               g109(.a(\a[18] ), .b(\b[17] ), .c(new_n198), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  aoai13aa1n04x5               g111(.a(new_n206), .b(new_n204), .c(new_n189), .d(new_n194), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g114(.a(\b[18] ), .b(\a[19] ), .o1(new_n210));
  nand02aa1d04x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  tech160nm_fixnrc02aa1n05x5   g116(.a(\b[19] ), .b(\a[20] ), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoi112aa1n03x4               g118(.a(new_n210), .b(new_n213), .c(new_n207), .d(new_n211), .o1(new_n214));
  inv000aa1d42x5               g119(.a(new_n210), .o1(new_n215));
  nona23aa1n09x5               g120(.a(new_n190), .b(new_n153), .c(new_n135), .d(new_n142), .out0(new_n216));
  aoai13aa1n06x5               g121(.a(new_n194), .b(new_n216), .c(new_n148), .d(new_n152), .o1(new_n217));
  nanb02aa1n12x5               g122(.a(new_n210), .b(new_n211), .out0(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n205), .c(new_n217), .d(new_n203), .o1(new_n220));
  aoi012aa1n03x5               g125(.a(new_n212), .b(new_n220), .c(new_n215), .o1(new_n221));
  norp02aa1n03x5               g126(.a(new_n221), .b(new_n214), .o1(\s[20] ));
  nona22aa1n03x5               g127(.a(new_n203), .b(new_n218), .c(new_n212), .out0(new_n223));
  nanp02aa1n02x5               g128(.a(\b[17] ), .b(\a[18] ), .o1(new_n224));
  oai022aa1n02x5               g129(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n225));
  nanp03aa1n02x5               g130(.a(new_n225), .b(new_n224), .c(new_n211), .o1(new_n226));
  oai022aa1n12x5               g131(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n227));
  aboi22aa1n12x5               g132(.a(new_n227), .b(new_n226), .c(\b[19] ), .d(\a[20] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n223), .c(new_n189), .d(new_n194), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n06x5               g136(.a(\b[20] ), .b(\a[21] ), .o1(new_n232));
  nand42aa1n02x5               g137(.a(\b[20] ), .b(\a[21] ), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(new_n234));
  inv040aa1d32x5               g139(.a(\a[22] ), .o1(new_n235));
  inv040aa1d28x5               g140(.a(\b[21] ), .o1(new_n236));
  nand02aa1d16x5               g141(.a(new_n236), .b(new_n235), .o1(new_n237));
  nand02aa1d28x5               g142(.a(\b[21] ), .b(\a[22] ), .o1(new_n238));
  nand02aa1n08x5               g143(.a(new_n237), .b(new_n238), .o1(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoi112aa1n03x4               g145(.a(new_n232), .b(new_n240), .c(new_n230), .d(new_n234), .o1(new_n241));
  inv030aa1n02x5               g146(.a(new_n232), .o1(new_n242));
  inv000aa1n02x5               g147(.a(new_n223), .o1(new_n243));
  aoai13aa1n03x5               g148(.a(new_n234), .b(new_n228), .c(new_n217), .d(new_n243), .o1(new_n244));
  tech160nm_fiaoi012aa1n02p5x5 g149(.a(new_n239), .b(new_n244), .c(new_n242), .o1(new_n245));
  norp02aa1n03x5               g150(.a(new_n245), .b(new_n241), .o1(\s[22] ));
  nano22aa1n03x5               g151(.a(new_n239), .b(new_n242), .c(new_n233), .out0(new_n247));
  nona23aa1d18x5               g152(.a(new_n203), .b(new_n247), .c(new_n212), .d(new_n218), .out0(new_n248));
  norp02aa1n02x5               g153(.a(\b[17] ), .b(\a[18] ), .o1(new_n249));
  nor002aa1n02x5               g154(.a(new_n249), .b(new_n197), .o1(new_n250));
  nano22aa1n03x7               g155(.a(new_n250), .b(new_n224), .c(new_n211), .out0(new_n251));
  nanp02aa1n02x5               g156(.a(\b[19] ), .b(\a[20] ), .o1(new_n252));
  nano32aa1n03x7               g157(.a(new_n239), .b(new_n242), .c(new_n233), .d(new_n252), .out0(new_n253));
  oaoi03aa1n02x5               g158(.a(new_n235), .b(new_n236), .c(new_n232), .o1(new_n254));
  inv000aa1n03x5               g159(.a(new_n254), .o1(new_n255));
  oaoi13aa1n12x5               g160(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n227), .o1(new_n256));
  aoai13aa1n04x5               g161(.a(new_n256), .b(new_n248), .c(new_n189), .d(new_n194), .o1(new_n257));
  xorb03aa1n02x5               g162(.a(new_n257), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor042aa1n03x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  xorc02aa1n06x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  xorc02aa1n12x5               g165(.a(\a[24] ), .b(\b[23] ), .out0(new_n261));
  aoi112aa1n03x4               g166(.a(new_n259), .b(new_n261), .c(new_n257), .d(new_n260), .o1(new_n262));
  inv000aa1n02x5               g167(.a(new_n259), .o1(new_n263));
  inv040aa1n03x5               g168(.a(new_n248), .o1(new_n264));
  inv000aa1n02x5               g169(.a(new_n256), .o1(new_n265));
  aoai13aa1n03x5               g170(.a(new_n260), .b(new_n265), .c(new_n217), .d(new_n264), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n261), .o1(new_n267));
  tech160nm_fiaoi012aa1n02p5x5 g172(.a(new_n267), .b(new_n266), .c(new_n263), .o1(new_n268));
  nor002aa1n02x5               g173(.a(new_n268), .b(new_n262), .o1(\s[24] ));
  nano32aa1n03x7               g174(.a(new_n223), .b(new_n261), .c(new_n247), .d(new_n260), .out0(new_n270));
  inv020aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  oai012aa1n06x5               g176(.a(new_n253), .b(new_n251), .c(new_n227), .o1(new_n272));
  nand22aa1n03x5               g177(.a(new_n261), .b(new_n260), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .c(new_n263), .carry(new_n274));
  aoai13aa1n02x5               g179(.a(new_n274), .b(new_n273), .c(new_n272), .d(new_n254), .o1(new_n275));
  inv000aa1n02x5               g180(.a(new_n275), .o1(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n271), .c(new_n189), .d(new_n194), .o1(new_n277));
  xorb03aa1n02x5               g182(.a(new_n277), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  tech160nm_fixorc02aa1n03p5x5 g184(.a(\a[25] ), .b(\b[24] ), .out0(new_n280));
  xorc02aa1n12x5               g185(.a(\a[26] ), .b(\b[25] ), .out0(new_n281));
  aoi112aa1n03x4               g186(.a(new_n279), .b(new_n281), .c(new_n277), .d(new_n280), .o1(new_n282));
  inv000aa1n02x5               g187(.a(new_n279), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n280), .b(new_n275), .c(new_n217), .d(new_n270), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n281), .o1(new_n285));
  tech160nm_fiaoi012aa1n02p5x5 g190(.a(new_n285), .b(new_n284), .c(new_n283), .o1(new_n286));
  nor002aa1n02x5               g191(.a(new_n286), .b(new_n282), .o1(\s[26] ));
  nano23aa1n02x4               g192(.a(new_n100), .b(new_n99), .c(new_n101), .d(new_n98), .out0(new_n288));
  nona22aa1n02x4               g193(.a(new_n288), .b(new_n150), .c(new_n149), .out0(new_n289));
  aoi012aa1n02x5               g194(.a(new_n118), .b(new_n147), .c(new_n146), .o1(new_n290));
  oai012aa1n03x5               g195(.a(new_n152), .b(new_n290), .c(new_n289), .o1(new_n291));
  nanp03aa1n02x5               g196(.a(new_n190), .b(new_n140), .c(new_n156), .o1(new_n292));
  nanb02aa1n02x5               g197(.a(new_n193), .b(new_n292), .out0(new_n293));
  inv000aa1n02x5               g198(.a(new_n273), .o1(new_n294));
  and002aa1n06x5               g199(.a(new_n281), .b(new_n280), .o(new_n295));
  nano22aa1d15x5               g200(.a(new_n248), .b(new_n294), .c(new_n295), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n293), .c(new_n291), .d(new_n188), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[26] ), .b(\b[25] ), .c(new_n283), .carry(new_n298));
  inv000aa1d42x5               g203(.a(new_n298), .o1(new_n299));
  tech160nm_fiaoi012aa1n05x5   g204(.a(new_n299), .b(new_n275), .c(new_n295), .o1(new_n300));
  xorc02aa1n12x5               g205(.a(\a[27] ), .b(\b[26] ), .out0(new_n301));
  xnbna2aa1n03x5               g206(.a(new_n301), .b(new_n297), .c(new_n300), .out0(\s[27] ));
  norp02aa1n02x5               g207(.a(\b[26] ), .b(\a[27] ), .o1(new_n303));
  inv040aa1n03x5               g208(.a(new_n303), .o1(new_n304));
  inv000aa1d42x5               g209(.a(new_n301), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n305), .b(new_n297), .c(new_n300), .o1(new_n306));
  xnrc02aa1n12x5               g211(.a(\b[27] ), .b(\a[28] ), .out0(new_n307));
  nano22aa1n03x5               g212(.a(new_n306), .b(new_n304), .c(new_n307), .out0(new_n308));
  nanb02aa1n02x5               g213(.a(new_n227), .b(new_n226), .out0(new_n309));
  aoai13aa1n02x5               g214(.a(new_n294), .b(new_n255), .c(new_n309), .d(new_n253), .o1(new_n310));
  inv000aa1d42x5               g215(.a(new_n295), .o1(new_n311));
  aoai13aa1n06x5               g216(.a(new_n298), .b(new_n311), .c(new_n310), .d(new_n274), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n301), .b(new_n312), .c(new_n217), .d(new_n296), .o1(new_n313));
  tech160nm_fiaoi012aa1n02p5x5 g218(.a(new_n307), .b(new_n313), .c(new_n304), .o1(new_n314));
  norp02aa1n03x5               g219(.a(new_n314), .b(new_n308), .o1(\s[28] ));
  xnrc02aa1n02x5               g220(.a(\b[28] ), .b(\a[29] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n301), .b(new_n307), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n312), .c(new_n217), .d(new_n296), .o1(new_n318));
  oao003aa1n02x5               g223(.a(\a[28] ), .b(\b[27] ), .c(new_n304), .carry(new_n319));
  tech160nm_fiaoi012aa1n02p5x5 g224(.a(new_n316), .b(new_n318), .c(new_n319), .o1(new_n320));
  inv000aa1n02x5               g225(.a(new_n317), .o1(new_n321));
  tech160nm_fiaoi012aa1n02p5x5 g226(.a(new_n321), .b(new_n297), .c(new_n300), .o1(new_n322));
  nano22aa1n03x5               g227(.a(new_n322), .b(new_n316), .c(new_n319), .out0(new_n323));
  norp02aa1n03x5               g228(.a(new_n320), .b(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n110), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1d15x5               g230(.a(new_n301), .b(new_n316), .c(new_n307), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n312), .c(new_n217), .d(new_n296), .o1(new_n327));
  oao003aa1n02x5               g232(.a(\a[29] ), .b(\b[28] ), .c(new_n319), .carry(new_n328));
  xnrc02aa1n02x5               g233(.a(\b[29] ), .b(\a[30] ), .out0(new_n329));
  tech160nm_fiaoi012aa1n02p5x5 g234(.a(new_n329), .b(new_n327), .c(new_n328), .o1(new_n330));
  inv000aa1d42x5               g235(.a(new_n326), .o1(new_n331));
  tech160nm_fiaoi012aa1n02p5x5 g236(.a(new_n331), .b(new_n297), .c(new_n300), .o1(new_n332));
  nano22aa1n03x5               g237(.a(new_n332), .b(new_n328), .c(new_n329), .out0(new_n333));
  norp02aa1n03x5               g238(.a(new_n330), .b(new_n333), .o1(\s[30] ));
  nona32aa1n02x4               g239(.a(new_n301), .b(new_n329), .c(new_n316), .d(new_n307), .out0(new_n335));
  tech160nm_fiaoi012aa1n02p5x5 g240(.a(new_n335), .b(new_n297), .c(new_n300), .o1(new_n336));
  oao003aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(new_n328), .carry(new_n337));
  xnrc02aa1n02x5               g242(.a(\b[30] ), .b(\a[31] ), .out0(new_n338));
  nano22aa1n03x5               g243(.a(new_n336), .b(new_n337), .c(new_n338), .out0(new_n339));
  inv000aa1n02x5               g244(.a(new_n335), .o1(new_n340));
  aoai13aa1n03x5               g245(.a(new_n340), .b(new_n312), .c(new_n217), .d(new_n296), .o1(new_n341));
  tech160nm_fiaoi012aa1n02p5x5 g246(.a(new_n338), .b(new_n341), .c(new_n337), .o1(new_n342));
  norp02aa1n03x5               g247(.a(new_n342), .b(new_n339), .o1(\s[31] ));
  xnrb03aa1n02x5               g248(.a(new_n112), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g249(.a(\a[3] ), .b(\b[2] ), .c(new_n112), .o1(new_n345));
  xorb03aa1n02x5               g250(.a(new_n345), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g251(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oaoi03aa1n02x5               g252(.a(\a[5] ), .b(\b[4] ), .c(new_n290), .o1(new_n348));
  xorb03aa1n02x5               g253(.a(new_n348), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aobi12aa1n02x5               g254(.a(new_n122), .b(new_n119), .c(new_n288), .out0(new_n350));
  xnrb03aa1n02x5               g255(.a(new_n350), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g256(.a(\a[7] ), .b(\b[6] ), .c(new_n350), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g258(.a(new_n126), .b(new_n148), .c(new_n152), .out0(\s[9] ));
endmodule


