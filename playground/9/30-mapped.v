// Benchmark "adder" written by ABC on Wed Jul 17 16:49:17 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n194, new_n195,
    new_n196, new_n197, new_n198, new_n199, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n210, new_n211, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n218, new_n219,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n237, new_n238, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n246, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n265, new_n266,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n278, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n296, new_n297,
    new_n298, new_n299, new_n300, new_n301, new_n302, new_n304, new_n305,
    new_n306, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n316, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n338, new_n339, new_n340, new_n342, new_n343, new_n345, new_n347,
    new_n348, new_n349, new_n351, new_n353, new_n354, new_n355, new_n357,
    new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  and002aa1n12x5               g002(.a(\b[0] ), .b(\a[1] ), .o(new_n98));
  oaoi03aa1n09x5               g003(.a(\a[2] ), .b(\b[1] ), .c(new_n98), .o1(new_n99));
  nor042aa1n03x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nanp02aa1n09x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  norb02aa1n06x5               g006(.a(new_n101), .b(new_n100), .out0(new_n102));
  xorc02aa1n12x5               g007(.a(\a[3] ), .b(\b[2] ), .out0(new_n103));
  nand23aa1n06x5               g008(.a(new_n99), .b(new_n103), .c(new_n102), .o1(new_n104));
  nor042aa1n02x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  aoi012aa1n06x5               g010(.a(new_n100), .b(new_n105), .c(new_n101), .o1(new_n106));
  nanp02aa1n09x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nor002aa1n16x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor002aa1n03x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  tech160nm_finand02aa1n05x5   g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nano23aa1n06x5               g015(.a(new_n109), .b(new_n108), .c(new_n110), .d(new_n107), .out0(new_n111));
  xorc02aa1n12x5               g016(.a(\a[8] ), .b(\b[7] ), .out0(new_n112));
  nor002aa1d32x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n06x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  norb02aa1n09x5               g019(.a(new_n114), .b(new_n113), .out0(new_n115));
  nand23aa1n03x5               g020(.a(new_n111), .b(new_n112), .c(new_n115), .o1(new_n116));
  nano22aa1n03x7               g021(.a(new_n113), .b(new_n107), .c(new_n114), .out0(new_n117));
  oai022aa1n03x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  inv000aa1n03x5               g023(.a(new_n113), .o1(new_n119));
  oaoi03aa1n06x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  aoi013aa1n06x4               g025(.a(new_n120), .b(new_n117), .c(new_n112), .d(new_n118), .o1(new_n121));
  aoai13aa1n12x5               g026(.a(new_n121), .b(new_n116), .c(new_n104), .d(new_n106), .o1(new_n122));
  xorc02aa1n12x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  nanp02aa1n06x5               g028(.a(new_n122), .b(new_n123), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[10] ), .b(\b[9] ), .out0(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  nand42aa1n16x5               g031(.a(\b[10] ), .b(\a[11] ), .o1(new_n127));
  nor002aa1n16x5               g032(.a(\b[10] ), .b(\a[11] ), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n127), .b(new_n128), .out0(new_n129));
  oai022aa1d24x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  aboi22aa1n03x5               g035(.a(new_n130), .b(new_n124), .c(\b[9] ), .d(\a[10] ), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n128), .o1(new_n132));
  nanb02aa1n03x5               g037(.a(new_n130), .b(new_n124), .out0(new_n133));
  aoi022aa1d24x5               g038(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n134));
  nand43aa1n02x5               g039(.a(new_n133), .b(new_n132), .c(new_n134), .o1(new_n135));
  oa0012aa1n03x5               g040(.a(new_n135), .b(new_n131), .c(new_n129), .o(\s[11] ));
  aob012aa1n02x5               g041(.a(new_n132), .b(new_n133), .c(new_n134), .out0(new_n137));
  norp02aa1n04x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand42aa1d28x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n02x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  aoib12aa1n02x5               g045(.a(new_n128), .b(new_n139), .c(new_n138), .out0(new_n141));
  aoi022aa1n03x5               g046(.a(new_n137), .b(new_n140), .c(new_n135), .d(new_n141), .o1(\s[12] ));
  nano23aa1d15x5               g047(.a(new_n138), .b(new_n128), .c(new_n139), .d(new_n127), .out0(new_n143));
  nand23aa1d12x5               g048(.a(new_n143), .b(new_n123), .c(new_n125), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  nanp02aa1n06x5               g050(.a(new_n122), .b(new_n145), .o1(new_n146));
  oai022aa1n09x5               g051(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n147));
  aoai13aa1n12x5               g052(.a(new_n139), .b(new_n147), .c(new_n130), .d(new_n134), .o1(new_n148));
  nanp02aa1n06x5               g053(.a(new_n146), .b(new_n148), .o1(new_n149));
  xorc02aa1n12x5               g054(.a(\a[13] ), .b(\b[12] ), .out0(new_n150));
  nano32aa1n02x4               g055(.a(new_n147), .b(new_n130), .c(new_n134), .d(new_n139), .out0(new_n151));
  aoi112aa1n02x5               g056(.a(new_n151), .b(new_n150), .c(new_n139), .d(new_n147), .o1(new_n152));
  aoi022aa1n02x5               g057(.a(new_n149), .b(new_n150), .c(new_n146), .d(new_n152), .o1(\s[13] ));
  norp02aa1n02x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  inv000aa1n02x5               g059(.a(new_n154), .o1(new_n155));
  inv000aa1d42x5               g060(.a(new_n148), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n150), .b(new_n156), .c(new_n122), .d(new_n145), .o1(new_n157));
  xorc02aa1n06x5               g062(.a(\a[14] ), .b(\b[13] ), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n157), .c(new_n155), .out0(\s[14] ));
  nand22aa1n09x5               g064(.a(new_n158), .b(new_n150), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  aoai13aa1n06x5               g066(.a(new_n161), .b(new_n156), .c(new_n122), .d(new_n145), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  norp02aa1n04x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n06x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n162), .c(new_n164), .out0(\s[15] ));
  aoai13aa1n03x5               g073(.a(new_n167), .b(new_n163), .c(new_n149), .d(new_n161), .o1(new_n169));
  inv030aa1n02x5               g074(.a(new_n165), .o1(new_n170));
  inv000aa1n02x5               g075(.a(new_n167), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n170), .b(new_n171), .c(new_n162), .d(new_n164), .o1(new_n172));
  norp02aa1n04x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1n04x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n174), .b(new_n173), .out0(new_n175));
  norp02aa1n02x5               g080(.a(new_n175), .b(new_n165), .o1(new_n176));
  aoi022aa1n03x5               g081(.a(new_n172), .b(new_n175), .c(new_n169), .d(new_n176), .o1(\s[16] ));
  nona23aa1d18x5               g082(.a(new_n174), .b(new_n166), .c(new_n165), .d(new_n173), .out0(new_n178));
  nor043aa1n04x5               g083(.a(new_n144), .b(new_n160), .c(new_n178), .o1(new_n179));
  nand42aa1n06x5               g084(.a(new_n122), .b(new_n179), .o1(new_n180));
  nanp02aa1n02x5               g085(.a(\b[13] ), .b(\a[14] ), .o1(new_n181));
  oai022aa1n02x5               g086(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n182));
  nanb03aa1n03x5               g087(.a(new_n173), .b(new_n174), .c(new_n166), .out0(new_n183));
  nano32aa1n03x5               g088(.a(new_n183), .b(new_n182), .c(new_n170), .d(new_n181), .out0(new_n184));
  aoi012aa1n02x5               g089(.a(new_n173), .b(new_n165), .c(new_n174), .o1(new_n185));
  norb02aa1n06x5               g090(.a(new_n185), .b(new_n184), .out0(new_n186));
  oai013aa1d12x5               g091(.a(new_n186), .b(new_n160), .c(new_n148), .d(new_n178), .o1(new_n187));
  inv020aa1n03x5               g092(.a(new_n187), .o1(new_n188));
  nanp02aa1n12x5               g093(.a(new_n180), .b(new_n188), .o1(new_n189));
  xorc02aa1n12x5               g094(.a(\a[17] ), .b(\b[16] ), .out0(new_n190));
  nona22aa1n02x4               g095(.a(new_n161), .b(new_n148), .c(new_n178), .out0(new_n191));
  nano23aa1n02x4               g096(.a(new_n190), .b(new_n184), .c(new_n191), .d(new_n185), .out0(new_n192));
  aoi022aa1n02x5               g097(.a(new_n189), .b(new_n190), .c(new_n192), .d(new_n180), .o1(\s[17] ));
  nor002aa1d32x5               g098(.a(\b[16] ), .b(\a[17] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  aoai13aa1n06x5               g100(.a(new_n190), .b(new_n187), .c(new_n122), .d(new_n179), .o1(new_n196));
  nor022aa1n16x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  nand02aa1d08x5               g102(.a(\b[17] ), .b(\a[18] ), .o1(new_n198));
  norb02aa1n06x4               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n199), .b(new_n196), .c(new_n195), .out0(\s[18] ));
  and002aa1n02x5               g105(.a(new_n190), .b(new_n199), .o(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n187), .c(new_n122), .d(new_n179), .o1(new_n202));
  oaoi03aa1n02x5               g107(.a(\a[18] ), .b(\b[17] ), .c(new_n195), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  nor002aa1d32x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nanp02aa1n12x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norb02aa1n12x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  xnbna2aa1n03x5               g112(.a(new_n207), .b(new_n202), .c(new_n204), .out0(\s[19] ));
  xnrc02aa1n02x5               g113(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g114(.a(new_n207), .b(new_n203), .c(new_n189), .d(new_n201), .o1(new_n210));
  inv030aa1n03x5               g115(.a(new_n205), .o1(new_n211));
  inv000aa1d42x5               g116(.a(new_n207), .o1(new_n212));
  aoai13aa1n02x5               g117(.a(new_n211), .b(new_n212), .c(new_n202), .d(new_n204), .o1(new_n213));
  nor002aa1n16x5               g118(.a(\b[19] ), .b(\a[20] ), .o1(new_n214));
  nand02aa1d28x5               g119(.a(\b[19] ), .b(\a[20] ), .o1(new_n215));
  norb02aa1n02x7               g120(.a(new_n215), .b(new_n214), .out0(new_n216));
  inv000aa1d42x5               g121(.a(\a[19] ), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\b[18] ), .o1(new_n218));
  aboi22aa1n03x5               g123(.a(new_n214), .b(new_n215), .c(new_n217), .d(new_n218), .out0(new_n219));
  aoi022aa1n03x5               g124(.a(new_n213), .b(new_n216), .c(new_n210), .d(new_n219), .o1(\s[20] ));
  nano32aa1n03x7               g125(.a(new_n212), .b(new_n190), .c(new_n216), .d(new_n199), .out0(new_n221));
  aoai13aa1n03x5               g126(.a(new_n221), .b(new_n187), .c(new_n122), .d(new_n179), .o1(new_n222));
  nanb03aa1n09x5               g127(.a(new_n214), .b(new_n215), .c(new_n206), .out0(new_n223));
  oai112aa1n06x5               g128(.a(new_n211), .b(new_n198), .c(new_n197), .d(new_n194), .o1(new_n224));
  aoi012aa1d18x5               g129(.a(new_n214), .b(new_n205), .c(new_n215), .o1(new_n225));
  oai012aa1d24x5               g130(.a(new_n225), .b(new_n224), .c(new_n223), .o1(new_n226));
  nor002aa1d32x5               g131(.a(\b[20] ), .b(\a[21] ), .o1(new_n227));
  nand42aa1n04x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  norb02aa1n12x5               g133(.a(new_n228), .b(new_n227), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n226), .c(new_n189), .d(new_n221), .o1(new_n230));
  nano22aa1n03x7               g135(.a(new_n214), .b(new_n206), .c(new_n215), .out0(new_n231));
  oai012aa1n02x7               g136(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .o1(new_n232));
  oab012aa1n04x5               g137(.a(new_n232), .b(new_n194), .c(new_n197), .out0(new_n233));
  inv020aa1n02x5               g138(.a(new_n225), .o1(new_n234));
  aoi112aa1n02x5               g139(.a(new_n234), .b(new_n229), .c(new_n233), .d(new_n231), .o1(new_n235));
  aobi12aa1n03x7               g140(.a(new_n230), .b(new_n235), .c(new_n222), .out0(\s[21] ));
  inv000aa1d42x5               g141(.a(new_n226), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n227), .o1(new_n238));
  inv000aa1d42x5               g143(.a(new_n229), .o1(new_n239));
  aoai13aa1n02x7               g144(.a(new_n238), .b(new_n239), .c(new_n222), .d(new_n237), .o1(new_n240));
  nor042aa1n04x5               g145(.a(\b[21] ), .b(\a[22] ), .o1(new_n241));
  nand02aa1n06x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n242), .b(new_n241), .out0(new_n243));
  aoib12aa1n02x5               g148(.a(new_n227), .b(new_n242), .c(new_n241), .out0(new_n244));
  aoi022aa1n03x5               g149(.a(new_n240), .b(new_n243), .c(new_n230), .d(new_n244), .o1(\s[22] ));
  inv020aa1n02x5               g150(.a(new_n221), .o1(new_n246));
  nano22aa1n03x7               g151(.a(new_n246), .b(new_n229), .c(new_n243), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n187), .c(new_n122), .d(new_n179), .o1(new_n248));
  nano23aa1n09x5               g153(.a(new_n227), .b(new_n241), .c(new_n242), .d(new_n228), .out0(new_n249));
  aoi012aa1n12x5               g154(.a(new_n241), .b(new_n227), .c(new_n242), .o1(new_n250));
  inv000aa1n02x5               g155(.a(new_n250), .o1(new_n251));
  aoi012aa1n06x5               g156(.a(new_n251), .b(new_n226), .c(new_n249), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n252), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[23] ), .b(\b[22] ), .out0(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n253), .c(new_n189), .d(new_n247), .o1(new_n255));
  aoi112aa1n02x5               g160(.a(new_n254), .b(new_n251), .c(new_n226), .d(new_n249), .o1(new_n256));
  aobi12aa1n02x7               g161(.a(new_n255), .b(new_n256), .c(new_n248), .out0(\s[23] ));
  nor042aa1n06x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  inv000aa1d42x5               g163(.a(new_n258), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n254), .o1(new_n260));
  aoai13aa1n02x7               g165(.a(new_n259), .b(new_n260), .c(new_n248), .d(new_n252), .o1(new_n261));
  xorc02aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  norp02aa1n02x5               g167(.a(new_n262), .b(new_n258), .o1(new_n263));
  aoi022aa1n03x5               g168(.a(new_n261), .b(new_n262), .c(new_n255), .d(new_n263), .o1(\s[24] ));
  and002aa1n06x5               g169(.a(new_n262), .b(new_n254), .o(new_n265));
  nano22aa1n06x5               g170(.a(new_n246), .b(new_n265), .c(new_n249), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n187), .c(new_n122), .d(new_n179), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n249), .b(new_n234), .c(new_n233), .d(new_n231), .o1(new_n268));
  inv000aa1n09x5               g173(.a(new_n265), .o1(new_n269));
  oao003aa1n02x5               g174(.a(\a[24] ), .b(\b[23] ), .c(new_n259), .carry(new_n270));
  aoai13aa1n12x5               g175(.a(new_n270), .b(new_n269), .c(new_n268), .d(new_n250), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[25] ), .b(\b[24] ), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n271), .c(new_n189), .d(new_n266), .o1(new_n273));
  aoai13aa1n02x5               g178(.a(new_n265), .b(new_n251), .c(new_n226), .d(new_n249), .o1(new_n274));
  inv000aa1d42x5               g179(.a(new_n272), .o1(new_n275));
  and003aa1n02x5               g180(.a(new_n274), .b(new_n275), .c(new_n270), .o(new_n276));
  aobi12aa1n03x7               g181(.a(new_n273), .b(new_n276), .c(new_n267), .out0(\s[25] ));
  inv000aa1d42x5               g182(.a(new_n271), .o1(new_n278));
  nor042aa1n03x5               g183(.a(\b[24] ), .b(\a[25] ), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  aoai13aa1n02x7               g185(.a(new_n280), .b(new_n275), .c(new_n267), .d(new_n278), .o1(new_n281));
  tech160nm_fixorc02aa1n02p5x5 g186(.a(\a[26] ), .b(\b[25] ), .out0(new_n282));
  norp02aa1n02x5               g187(.a(new_n282), .b(new_n279), .o1(new_n283));
  aoi022aa1n02x7               g188(.a(new_n281), .b(new_n282), .c(new_n273), .d(new_n283), .o1(\s[26] ));
  and002aa1n12x5               g189(.a(new_n282), .b(new_n272), .o(new_n285));
  nano32aa1n03x7               g190(.a(new_n246), .b(new_n285), .c(new_n249), .d(new_n265), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n187), .c(new_n122), .d(new_n179), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n285), .o1(new_n288));
  oao003aa1n12x5               g193(.a(\a[26] ), .b(\b[25] ), .c(new_n280), .carry(new_n289));
  aoai13aa1n04x5               g194(.a(new_n289), .b(new_n288), .c(new_n274), .d(new_n270), .o1(new_n290));
  xorc02aa1n12x5               g195(.a(\a[27] ), .b(\b[26] ), .out0(new_n291));
  aoai13aa1n06x5               g196(.a(new_n291), .b(new_n290), .c(new_n189), .d(new_n286), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n289), .o1(new_n293));
  aoi112aa1n02x5               g198(.a(new_n291), .b(new_n293), .c(new_n271), .d(new_n285), .o1(new_n294));
  aobi12aa1n02x7               g199(.a(new_n292), .b(new_n294), .c(new_n287), .out0(\s[27] ));
  tech160nm_fiaoi012aa1n05x5   g200(.a(new_n293), .b(new_n271), .c(new_n285), .o1(new_n296));
  nor042aa1n06x5               g201(.a(\b[26] ), .b(\a[27] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n291), .o1(new_n299));
  aoai13aa1n02x7               g204(.a(new_n298), .b(new_n299), .c(new_n296), .d(new_n287), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[28] ), .b(\b[27] ), .out0(new_n301));
  norp02aa1n02x5               g206(.a(new_n301), .b(new_n297), .o1(new_n302));
  aoi022aa1n02x7               g207(.a(new_n300), .b(new_n301), .c(new_n292), .d(new_n302), .o1(\s[28] ));
  inv000aa1d42x5               g208(.a(\a[27] ), .o1(new_n304));
  inv000aa1d42x5               g209(.a(\a[28] ), .o1(new_n305));
  xroi22aa1d04x5               g210(.a(new_n304), .b(\b[26] ), .c(new_n305), .d(\b[27] ), .out0(new_n306));
  aoai13aa1n03x5               g211(.a(new_n306), .b(new_n290), .c(new_n189), .d(new_n286), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n306), .o1(new_n308));
  inv000aa1d42x5               g213(.a(\b[27] ), .o1(new_n309));
  oaoi03aa1n02x5               g214(.a(new_n305), .b(new_n309), .c(new_n297), .o1(new_n310));
  aoai13aa1n02x7               g215(.a(new_n310), .b(new_n308), .c(new_n296), .d(new_n287), .o1(new_n311));
  xorc02aa1n02x5               g216(.a(\a[29] ), .b(\b[28] ), .out0(new_n312));
  norb02aa1n02x5               g217(.a(new_n310), .b(new_n312), .out0(new_n313));
  aoi022aa1n03x5               g218(.a(new_n311), .b(new_n312), .c(new_n307), .d(new_n313), .o1(\s[29] ));
  xnrb03aa1n02x5               g219(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g220(.a(new_n299), .b(new_n301), .c(new_n312), .out0(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n290), .c(new_n189), .d(new_n286), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  inv000aa1d42x5               g223(.a(\a[29] ), .o1(new_n319));
  inv000aa1d42x5               g224(.a(\b[28] ), .o1(new_n320));
  aoi012aa1n02x5               g225(.a(new_n297), .b(new_n305), .c(new_n309), .o1(new_n321));
  aoi122aa1n02x5               g226(.a(new_n321), .b(\b[28] ), .c(\a[29] ), .d(\b[27] ), .e(\a[28] ), .o1(new_n322));
  aoi012aa1n02x5               g227(.a(new_n322), .b(new_n319), .c(new_n320), .o1(new_n323));
  aoai13aa1n02x7               g228(.a(new_n323), .b(new_n318), .c(new_n296), .d(new_n287), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  aoi112aa1n02x5               g230(.a(new_n322), .b(new_n325), .c(new_n319), .d(new_n320), .o1(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n317), .d(new_n326), .o1(\s[30] ));
  nano32aa1n02x4               g232(.a(new_n299), .b(new_n325), .c(new_n301), .d(new_n312), .out0(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n290), .c(new_n189), .d(new_n286), .o1(new_n329));
  xorc02aa1n02x5               g234(.a(\a[31] ), .b(\b[30] ), .out0(new_n330));
  nanp02aa1n02x5               g235(.a(\b[29] ), .b(\a[30] ), .o1(new_n331));
  oai022aa1n02x5               g236(.a(\a[29] ), .b(\b[28] ), .c(\b[29] ), .d(\a[30] ), .o1(new_n332));
  oaoi13aa1n02x5               g237(.a(new_n330), .b(new_n331), .c(new_n322), .d(new_n332), .o1(new_n333));
  inv000aa1n02x5               g238(.a(new_n328), .o1(new_n334));
  oai012aa1n02x5               g239(.a(new_n331), .b(new_n322), .c(new_n332), .o1(new_n335));
  aoai13aa1n02x7               g240(.a(new_n335), .b(new_n334), .c(new_n296), .d(new_n287), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n336), .b(new_n330), .c(new_n329), .d(new_n333), .o1(\s[31] ));
  orn002aa1n02x5               g242(.a(\a[2] ), .b(\b[1] ), .o(new_n338));
  nanp02aa1n02x5               g243(.a(\b[1] ), .b(\a[2] ), .o1(new_n339));
  nanb03aa1n02x5               g244(.a(new_n98), .b(new_n338), .c(new_n339), .out0(new_n340));
  xnbna2aa1n03x5               g245(.a(new_n103), .b(new_n340), .c(new_n338), .out0(\s[3] ));
  nanp02aa1n02x5               g246(.a(new_n104), .b(new_n106), .o1(new_n342));
  aoi112aa1n02x5               g247(.a(new_n105), .b(new_n102), .c(new_n99), .d(new_n103), .o1(new_n343));
  oaoi13aa1n02x5               g248(.a(new_n343), .b(new_n342), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  norb02aa1n02x5               g249(.a(new_n110), .b(new_n109), .out0(new_n345));
  xnbna2aa1n03x5               g250(.a(new_n345), .b(new_n104), .c(new_n106), .out0(\s[5] ));
  norb02aa1n02x5               g251(.a(new_n107), .b(new_n108), .out0(new_n347));
  aoai13aa1n02x5               g252(.a(new_n347), .b(new_n109), .c(new_n342), .d(new_n110), .o1(new_n348));
  aoi112aa1n02x5               g253(.a(new_n109), .b(new_n347), .c(new_n342), .d(new_n345), .o1(new_n349));
  norb02aa1n02x5               g254(.a(new_n348), .b(new_n349), .out0(\s[6] ));
  inv000aa1d42x5               g255(.a(new_n108), .o1(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n115), .b(new_n348), .c(new_n351), .out0(\s[7] ));
  aob012aa1n02x5               g257(.a(new_n115), .b(new_n348), .c(new_n351), .out0(new_n353));
  nanp02aa1n02x5               g258(.a(new_n353), .b(new_n119), .o1(new_n354));
  norp02aa1n02x5               g259(.a(new_n112), .b(new_n113), .o1(new_n355));
  aoi022aa1n02x5               g260(.a(new_n354), .b(new_n112), .c(new_n353), .d(new_n355), .o1(\s[8] ));
  aoi012aa1n02x5               g261(.a(new_n116), .b(new_n104), .c(new_n106), .o1(new_n357));
  aoi113aa1n02x5               g262(.a(new_n123), .b(new_n120), .c(new_n117), .d(new_n112), .e(new_n118), .o1(new_n358));
  aboi22aa1n03x5               g263(.a(new_n357), .b(new_n358), .c(new_n122), .d(new_n123), .out0(\s[9] ));
endmodule


