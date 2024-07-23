// Benchmark "adder" written by ABC on Wed Jul 17 13:57:22 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n156,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n187, new_n188,
    new_n189, new_n190, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n201, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n238, new_n239, new_n240, new_n241, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n305, new_n308, new_n310, new_n311, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n04x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n16x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  norp02aa1n02x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  nor042aa1n03x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  tech160nm_finand02aa1n05x5   g006(.a(\b[7] ), .b(\a[8] ), .o1(new_n102));
  norb02aa1n06x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nor042aa1n02x5               g008(.a(\b[6] ), .b(\a[7] ), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  norb02aa1n03x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  nor002aa1n12x5               g011(.a(\b[5] ), .b(\a[6] ), .o1(new_n107));
  nand42aa1n03x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nor022aa1n16x5               g013(.a(\b[4] ), .b(\a[5] ), .o1(new_n109));
  nand42aa1n03x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nona23aa1n09x5               g015(.a(new_n110), .b(new_n108), .c(new_n107), .d(new_n109), .out0(new_n111));
  nano22aa1n03x7               g016(.a(new_n111), .b(new_n103), .c(new_n106), .out0(new_n112));
  and002aa1n02x5               g017(.a(\b[3] ), .b(\a[4] ), .o(new_n113));
  inv040aa1d32x5               g018(.a(\a[3] ), .o1(new_n114));
  inv040aa1d32x5               g019(.a(\b[2] ), .o1(new_n115));
  nand42aa1n06x5               g020(.a(new_n115), .b(new_n114), .o1(new_n116));
  tech160nm_finand02aa1n03p5x5 g021(.a(\b[2] ), .b(\a[3] ), .o1(new_n117));
  nand02aa1d04x5               g022(.a(new_n116), .b(new_n117), .o1(new_n118));
  nor042aa1n12x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  nand42aa1d28x5               g024(.a(\b[0] ), .b(\a[1] ), .o1(new_n120));
  nand42aa1d28x5               g025(.a(\b[1] ), .b(\a[2] ), .o1(new_n121));
  aoi012aa1d24x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n122));
  oa0022aa1n02x5               g027(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n123));
  oaoi13aa1n12x5               g028(.a(new_n113), .b(new_n123), .c(new_n122), .d(new_n118), .o1(new_n124));
  aoi112aa1n02x5               g029(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n125));
  aoi112aa1n06x5               g030(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n126));
  oai112aa1n02x7               g031(.a(new_n106), .b(new_n103), .c(new_n126), .d(new_n107), .o1(new_n127));
  nona22aa1n06x5               g032(.a(new_n127), .b(new_n125), .c(new_n101), .out0(new_n128));
  tech160nm_fiao0012aa1n02p5x5 g033(.a(new_n128), .b(new_n124), .c(new_n112), .o(new_n129));
  nand42aa1n06x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  aoi012aa1n02x5               g035(.a(new_n100), .b(new_n129), .c(new_n130), .o1(new_n131));
  xnrc02aa1n02x5               g036(.a(new_n131), .b(new_n99), .out0(\s[10] ));
  nano23aa1n06x5               g037(.a(new_n97), .b(new_n100), .c(new_n130), .d(new_n98), .out0(new_n133));
  aoai13aa1n02x5               g038(.a(new_n133), .b(new_n128), .c(new_n124), .d(new_n112), .o1(new_n134));
  aoi012aa1n02x5               g039(.a(new_n97), .b(new_n100), .c(new_n98), .o1(new_n135));
  nor002aa1d32x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1d24x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norb02aa1d27x5               g042(.a(new_n137), .b(new_n136), .out0(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n136), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n138), .o1(new_n141));
  aoai13aa1n02x5               g046(.a(new_n140), .b(new_n141), .c(new_n134), .d(new_n135), .o1(new_n142));
  xorb03aa1n02x5               g047(.a(new_n142), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  aoi112aa1n03x5               g049(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n145));
  aoi112aa1n09x5               g050(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n146));
  tech160nm_finand02aa1n03p5x5 g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n06x5               g052(.a(new_n147), .b(new_n144), .out0(new_n148));
  oai112aa1n06x5               g053(.a(new_n138), .b(new_n148), .c(new_n146), .d(new_n97), .o1(new_n149));
  nona22aa1d24x5               g054(.a(new_n149), .b(new_n145), .c(new_n144), .out0(new_n150));
  inv000aa1d42x5               g055(.a(new_n150), .o1(new_n151));
  and003aa1n02x5               g056(.a(new_n133), .b(new_n148), .c(new_n138), .o(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n128), .c(new_n124), .d(new_n112), .o1(new_n153));
  nanp02aa1n02x5               g058(.a(new_n153), .b(new_n151), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor022aa1n08x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nanp02aa1n04x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  aoi012aa1n02x5               g062(.a(new_n156), .b(new_n154), .c(new_n157), .o1(new_n158));
  xnrb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n12x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nanp02aa1n04x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  aoi012aa1n02x5               g066(.a(new_n160), .b(new_n156), .c(new_n161), .o1(new_n162));
  nona23aa1n03x5               g067(.a(new_n161), .b(new_n157), .c(new_n156), .d(new_n160), .out0(new_n163));
  aoai13aa1n04x5               g068(.a(new_n162), .b(new_n163), .c(new_n153), .d(new_n151), .o1(new_n164));
  xorb03aa1n02x5               g069(.a(new_n164), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  nanp02aa1n09x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  nor002aa1d24x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nanp02aa1n09x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n12x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  aoi112aa1n02x5               g076(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n172));
  aoai13aa1n04x5               g077(.a(new_n171), .b(new_n166), .c(new_n164), .d(new_n167), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  nano23aa1n06x5               g079(.a(new_n156), .b(new_n160), .c(new_n161), .d(new_n157), .out0(new_n175));
  nano23aa1n06x5               g080(.a(new_n166), .b(new_n168), .c(new_n169), .d(new_n167), .out0(new_n176));
  nanp02aa1n02x5               g081(.a(new_n176), .b(new_n175), .o1(new_n177));
  nano32aa1n03x7               g082(.a(new_n177), .b(new_n133), .c(new_n138), .d(new_n148), .out0(new_n178));
  aoai13aa1n09x5               g083(.a(new_n178), .b(new_n128), .c(new_n124), .d(new_n112), .o1(new_n179));
  nanb02aa1n02x5               g084(.a(new_n166), .b(new_n167), .out0(new_n180));
  nor043aa1n02x5               g085(.a(new_n163), .b(new_n170), .c(new_n180), .o1(new_n181));
  aoi112aa1n02x5               g086(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n182));
  obai22aa1n03x5               g087(.a(new_n176), .b(new_n162), .c(\a[16] ), .d(\b[15] ), .out0(new_n183));
  aoi112aa1n09x5               g088(.a(new_n183), .b(new_n182), .c(new_n150), .d(new_n181), .o1(new_n184));
  nand02aa1d06x5               g089(.a(new_n184), .b(new_n179), .o1(new_n185));
  xorb03aa1n02x5               g090(.a(new_n185), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g091(.a(\a[18] ), .o1(new_n187));
  inv000aa1d42x5               g092(.a(\a[17] ), .o1(new_n188));
  inv000aa1d42x5               g093(.a(\b[16] ), .o1(new_n189));
  oaoi03aa1n03x5               g094(.a(new_n188), .b(new_n189), .c(new_n185), .o1(new_n190));
  xorb03aa1n02x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n187), .out0(\s[18] ));
  xroi22aa1d04x5               g096(.a(new_n188), .b(\b[16] ), .c(new_n187), .d(\b[17] ), .out0(new_n192));
  inv020aa1n02x5               g097(.a(new_n192), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\b[17] ), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  oao003aa1n02x5               g100(.a(new_n187), .b(new_n194), .c(new_n195), .carry(new_n196));
  inv000aa1d42x5               g101(.a(new_n196), .o1(new_n197));
  aoai13aa1n06x5               g102(.a(new_n197), .b(new_n193), .c(new_n184), .d(new_n179), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g104(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n03x5               g105(.a(\b[18] ), .b(\a[19] ), .o1(new_n201));
  nand42aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nor042aa1n04x5               g107(.a(\b[19] ), .b(\a[20] ), .o1(new_n203));
  nanp02aa1n06x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  norb02aa1n06x5               g109(.a(new_n204), .b(new_n203), .out0(new_n205));
  aoi112aa1n02x7               g110(.a(new_n201), .b(new_n205), .c(new_n198), .d(new_n202), .o1(new_n206));
  aoai13aa1n02x7               g111(.a(new_n205), .b(new_n201), .c(new_n198), .d(new_n202), .o1(new_n207));
  norb02aa1n03x4               g112(.a(new_n207), .b(new_n206), .out0(\s[20] ));
  nano23aa1n02x4               g113(.a(new_n201), .b(new_n203), .c(new_n204), .d(new_n202), .out0(new_n209));
  nanp02aa1n02x5               g114(.a(new_n192), .b(new_n209), .o1(new_n210));
  aoi112aa1n09x5               g115(.a(\b[18] ), .b(\a[19] ), .c(\a[20] ), .d(\b[19] ), .o1(new_n211));
  nor002aa1n02x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  aoi112aa1n03x5               g117(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n213));
  norb02aa1n06x4               g118(.a(new_n202), .b(new_n201), .out0(new_n214));
  oai112aa1n06x5               g119(.a(new_n214), .b(new_n205), .c(new_n213), .d(new_n212), .o1(new_n215));
  nona22aa1d18x5               g120(.a(new_n215), .b(new_n211), .c(new_n203), .out0(new_n216));
  inv000aa1d42x5               g121(.a(new_n216), .o1(new_n217));
  aoai13aa1n03x5               g122(.a(new_n217), .b(new_n210), .c(new_n184), .d(new_n179), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n08x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  nanp02aa1n03x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  norp02aa1n03x5               g126(.a(\b[21] ), .b(\a[22] ), .o1(new_n222));
  nanp02aa1n04x5               g127(.a(\b[21] ), .b(\a[22] ), .o1(new_n223));
  norb02aa1n02x5               g128(.a(new_n223), .b(new_n222), .out0(new_n224));
  aoi112aa1n02x7               g129(.a(new_n220), .b(new_n224), .c(new_n218), .d(new_n221), .o1(new_n225));
  aoai13aa1n02x7               g130(.a(new_n224), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n226));
  norb02aa1n03x4               g131(.a(new_n226), .b(new_n225), .out0(\s[22] ));
  nona23aa1n09x5               g132(.a(new_n223), .b(new_n221), .c(new_n220), .d(new_n222), .out0(new_n228));
  inv020aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  tech160nm_fiaoi012aa1n04x5   g134(.a(new_n222), .b(new_n220), .c(new_n223), .o1(new_n230));
  aobi12aa1n06x5               g135(.a(new_n230), .b(new_n216), .c(new_n229), .out0(new_n231));
  nano22aa1n03x7               g136(.a(new_n193), .b(new_n229), .c(new_n209), .out0(new_n232));
  inv000aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n03x5               g138(.a(new_n231), .b(new_n233), .c(new_n184), .d(new_n179), .o1(new_n234));
  xorc02aa1n12x5               g139(.a(\a[23] ), .b(\b[22] ), .out0(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n185), .c(new_n232), .o1(new_n236));
  aoi022aa1n02x5               g141(.a(new_n236), .b(new_n231), .c(new_n234), .d(new_n235), .o1(\s[23] ));
  norp02aa1n02x5               g142(.a(\b[22] ), .b(\a[23] ), .o1(new_n238));
  tech160nm_fixorc02aa1n03p5x5 g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  aoi112aa1n02x7               g144(.a(new_n238), .b(new_n239), .c(new_n234), .d(new_n235), .o1(new_n240));
  aoai13aa1n02x7               g145(.a(new_n239), .b(new_n238), .c(new_n234), .d(new_n235), .o1(new_n241));
  norb02aa1n03x4               g146(.a(new_n241), .b(new_n240), .out0(\s[24] ));
  and002aa1n02x5               g147(.a(new_n239), .b(new_n235), .o(new_n243));
  nanb03aa1n02x5               g148(.a(new_n210), .b(new_n243), .c(new_n229), .out0(new_n244));
  norp02aa1n02x5               g149(.a(\b[23] ), .b(\a[24] ), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n246));
  nanb03aa1n02x5               g151(.a(new_n230), .b(new_n239), .c(new_n235), .out0(new_n247));
  nona22aa1n03x5               g152(.a(new_n247), .b(new_n246), .c(new_n245), .out0(new_n248));
  nano22aa1n03x7               g153(.a(new_n228), .b(new_n235), .c(new_n239), .out0(new_n249));
  aoi012aa1n02x5               g154(.a(new_n248), .b(new_n216), .c(new_n249), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n250), .b(new_n244), .c(new_n184), .d(new_n179), .o1(new_n251));
  xorb03aa1n02x5               g156(.a(new_n251), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  tech160nm_fixorc02aa1n02p5x5 g158(.a(\a[25] ), .b(\b[24] ), .out0(new_n254));
  xorc02aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .out0(new_n255));
  aoi112aa1n03x5               g160(.a(new_n253), .b(new_n255), .c(new_n251), .d(new_n254), .o1(new_n256));
  aoai13aa1n02x7               g161(.a(new_n255), .b(new_n253), .c(new_n251), .d(new_n254), .o1(new_n257));
  norb02aa1n03x4               g162(.a(new_n257), .b(new_n256), .out0(\s[26] ));
  and002aa1n02x5               g163(.a(new_n255), .b(new_n254), .o(new_n259));
  nano32aa1n03x7               g164(.a(new_n210), .b(new_n259), .c(new_n229), .d(new_n243), .out0(new_n260));
  norp02aa1n02x5               g165(.a(\b[25] ), .b(\a[26] ), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n259), .b(new_n248), .c(new_n216), .d(new_n249), .o1(new_n263));
  nona22aa1d18x5               g168(.a(new_n263), .b(new_n262), .c(new_n261), .out0(new_n264));
  aoi012aa1n06x5               g169(.a(new_n264), .b(new_n185), .c(new_n260), .o1(new_n265));
  nor042aa1n04x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  inv000aa1n06x5               g171(.a(new_n266), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(\b[26] ), .b(\a[27] ), .o1(new_n268));
  xnbna2aa1n03x5               g173(.a(new_n265), .b(new_n268), .c(new_n267), .out0(\s[27] ));
  xorc02aa1n12x5               g174(.a(\a[28] ), .b(\b[27] ), .out0(new_n270));
  inv000aa1d42x5               g175(.a(new_n270), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n268), .b(new_n264), .c(new_n185), .d(new_n260), .o1(new_n272));
  aoi012aa1n03x5               g177(.a(new_n271), .b(new_n272), .c(new_n267), .o1(new_n273));
  nona22aa1n03x5               g178(.a(new_n272), .b(new_n270), .c(new_n266), .out0(new_n274));
  norb02aa1n02x7               g179(.a(new_n274), .b(new_n273), .out0(\s[28] ));
  nano22aa1n02x4               g180(.a(new_n271), .b(new_n267), .c(new_n268), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n264), .c(new_n185), .d(new_n260), .o1(new_n277));
  oaoi03aa1n09x5               g182(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .o1(new_n278));
  inv000aa1n06x5               g183(.a(new_n278), .o1(new_n279));
  xorc02aa1n12x5               g184(.a(\a[29] ), .b(\b[28] ), .out0(new_n280));
  inv000aa1d42x5               g185(.a(new_n280), .o1(new_n281));
  aoi012aa1n03x5               g186(.a(new_n281), .b(new_n277), .c(new_n279), .o1(new_n282));
  nona22aa1n03x5               g187(.a(new_n277), .b(new_n278), .c(new_n280), .out0(new_n283));
  norb02aa1n03x4               g188(.a(new_n283), .b(new_n282), .out0(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n120), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1n02x4               g190(.a(new_n281), .b(new_n270), .c(new_n268), .d(new_n267), .out0(new_n286));
  aoai13aa1n06x5               g191(.a(new_n286), .b(new_n264), .c(new_n185), .d(new_n260), .o1(new_n287));
  oao003aa1n09x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xorc02aa1n12x5               g193(.a(\a[30] ), .b(\b[29] ), .out0(new_n289));
  inv000aa1d42x5               g194(.a(new_n289), .o1(new_n290));
  aoi012aa1n03x5               g195(.a(new_n290), .b(new_n287), .c(new_n288), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n288), .o1(new_n292));
  nona22aa1n03x5               g197(.a(new_n287), .b(new_n292), .c(new_n289), .out0(new_n293));
  norb02aa1n02x7               g198(.a(new_n293), .b(new_n291), .out0(\s[30] ));
  xnrc02aa1n02x5               g199(.a(\b[30] ), .b(\a[31] ), .out0(new_n295));
  inv000aa1d42x5               g200(.a(new_n295), .o1(new_n296));
  and003aa1n03x5               g201(.a(new_n276), .b(new_n289), .c(new_n280), .o(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n264), .c(new_n185), .d(new_n260), .o1(new_n298));
  oaoi03aa1n12x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .o1(new_n299));
  nona22aa1n02x4               g204(.a(new_n298), .b(new_n299), .c(new_n296), .out0(new_n300));
  inv000aa1d42x5               g205(.a(new_n299), .o1(new_n301));
  aoi012aa1n03x5               g206(.a(new_n295), .b(new_n298), .c(new_n301), .o1(new_n302));
  norb02aa1n03x4               g207(.a(new_n300), .b(new_n302), .out0(\s[31] ));
  xnbna2aa1n03x5               g208(.a(new_n122), .b(new_n116), .c(new_n117), .out0(\s[3] ));
  oaoi03aa1n02x5               g209(.a(\a[3] ), .b(\b[2] ), .c(new_n122), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g211(.a(new_n124), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g212(.a(new_n109), .b(new_n124), .c(new_n110), .o1(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoi012aa1n02x5               g214(.a(new_n107), .b(new_n109), .c(new_n108), .o1(new_n310));
  oaib12aa1n02x5               g215(.a(new_n310), .b(new_n111), .c(new_n124), .out0(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g217(.a(new_n104), .b(new_n311), .c(new_n105), .o1(new_n313));
  xnrc02aa1n02x5               g218(.a(new_n313), .b(new_n103), .out0(\s[8] ));
  xorb03aa1n02x5               g219(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


