// Benchmark "adder" written by ABC on Wed Jul 17 22:35:53 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n153, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n233, new_n234, new_n235, new_n236, new_n237,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n302,
    new_n304, new_n307, new_n309, new_n310, new_n312, new_n314;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n02x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1d18x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  nand02aa1d12x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  tech160nm_finand02aa1n05x5   g007(.a(\b[1] ), .b(\a[2] ), .o1(new_n103));
  nor002aa1d32x5               g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nanp02aa1n04x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  oai112aa1n06x5               g010(.a(new_n105), .b(new_n103), .c(new_n104), .d(new_n102), .o1(new_n106));
  oa0022aa1n06x5               g011(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n107));
  aoi022aa1n12x5               g012(.a(new_n106), .b(new_n107), .c(\b[3] ), .d(\a[4] ), .o1(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand22aa1n12x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand02aa1d08x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nanb03aa1n12x5               g017(.a(new_n111), .b(new_n112), .c(new_n110), .out0(new_n113));
  nor022aa1n12x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor022aa1n16x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand42aa1n03x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  nona23aa1n03x5               g022(.a(new_n117), .b(new_n115), .c(new_n114), .d(new_n116), .out0(new_n118));
  nor043aa1n03x5               g023(.a(new_n118), .b(new_n113), .c(new_n109), .o1(new_n119));
  oai012aa1n02x5               g024(.a(new_n115), .b(new_n116), .c(new_n114), .o1(new_n120));
  aoi012aa1n02x7               g025(.a(new_n109), .b(new_n111), .c(new_n110), .o1(new_n121));
  oai012aa1n06x5               g026(.a(new_n121), .b(new_n113), .c(new_n120), .o1(new_n122));
  nand22aa1n09x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n15x5               g028(.a(new_n123), .b(new_n100), .out0(new_n124));
  aoai13aa1n03x5               g029(.a(new_n124), .b(new_n122), .c(new_n119), .d(new_n108), .o1(new_n125));
  xnbna2aa1n03x5               g030(.a(new_n99), .b(new_n125), .c(new_n101), .out0(\s[10] ));
  inv000aa1d42x5               g031(.a(\b[10] ), .o1(new_n127));
  nanb02aa1d36x5               g032(.a(\a[11] ), .b(new_n127), .out0(new_n128));
  nand42aa1d28x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(new_n128), .b(new_n129), .o1(new_n130));
  nona22aa1n02x4               g035(.a(new_n125), .b(new_n100), .c(new_n97), .out0(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n130), .b(new_n131), .c(new_n98), .out0(\s[11] ));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  aoi013aa1n03x5               g038(.a(new_n133), .b(new_n131), .c(new_n129), .d(new_n98), .o1(new_n134));
  xnrb03aa1n03x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norb03aa1d15x5               g040(.a(new_n98), .b(new_n97), .c(new_n133), .out0(new_n136));
  nor002aa1d32x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nano22aa1n09x5               g043(.a(new_n137), .b(new_n129), .c(new_n138), .out0(new_n139));
  nand23aa1d12x5               g044(.a(new_n139), .b(new_n136), .c(new_n124), .o1(new_n140));
  inv000aa1n02x5               g045(.a(new_n140), .o1(new_n141));
  aoai13aa1n03x5               g046(.a(new_n141), .b(new_n122), .c(new_n119), .d(new_n108), .o1(new_n142));
  nanb03aa1d18x5               g047(.a(new_n137), .b(new_n138), .c(new_n129), .out0(new_n143));
  oai022aa1d24x5               g048(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n144));
  nand03aa1d16x5               g049(.a(new_n144), .b(new_n128), .c(new_n98), .o1(new_n145));
  aoi012aa1n12x5               g050(.a(new_n137), .b(new_n133), .c(new_n138), .o1(new_n146));
  oai012aa1d24x5               g051(.a(new_n146), .b(new_n145), .c(new_n143), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nor042aa1n04x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nand42aa1d28x5               g054(.a(\b[12] ), .b(\a[13] ), .o1(new_n150));
  norb02aa1n02x5               g055(.a(new_n150), .b(new_n149), .out0(new_n151));
  xnbna2aa1n03x5               g056(.a(new_n151), .b(new_n142), .c(new_n148), .out0(\s[13] ));
  nanp02aa1n03x5               g057(.a(new_n142), .b(new_n148), .o1(new_n153));
  aoi012aa1n02x5               g058(.a(new_n149), .b(new_n153), .c(new_n150), .o1(new_n154));
  xnrb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nand42aa1n16x5               g061(.a(\b[13] ), .b(\a[14] ), .o1(new_n157));
  nano23aa1d15x5               g062(.a(new_n149), .b(new_n156), .c(new_n157), .d(new_n150), .out0(new_n158));
  oa0012aa1n06x5               g063(.a(new_n157), .b(new_n156), .c(new_n149), .o(new_n159));
  nor002aa1n20x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nanp02aa1n09x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n159), .c(new_n153), .d(new_n158), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n162), .b(new_n159), .c(new_n153), .d(new_n158), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  inv000aa1d42x5               g070(.a(new_n160), .o1(new_n166));
  nor042aa1n06x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nand02aa1d24x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  norb02aa1n02x5               g073(.a(new_n168), .b(new_n167), .out0(new_n169));
  xnbna2aa1n03x5               g074(.a(new_n169), .b(new_n163), .c(new_n166), .out0(\s[16] ));
  nano23aa1d15x5               g075(.a(new_n160), .b(new_n167), .c(new_n168), .d(new_n161), .out0(new_n171));
  nano22aa1n03x7               g076(.a(new_n140), .b(new_n158), .c(new_n171), .out0(new_n172));
  aoai13aa1n12x5               g077(.a(new_n172), .b(new_n122), .c(new_n119), .d(new_n108), .o1(new_n173));
  aoai13aa1n12x5               g078(.a(new_n171), .b(new_n159), .c(new_n147), .d(new_n158), .o1(new_n174));
  aoi012aa1d24x5               g079(.a(new_n167), .b(new_n160), .c(new_n168), .o1(new_n175));
  nanp03aa1d24x5               g080(.a(new_n173), .b(new_n174), .c(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g082(.a(\a[18] ), .o1(new_n178));
  inv040aa1d32x5               g083(.a(\a[17] ), .o1(new_n179));
  inv000aa1d42x5               g084(.a(\b[16] ), .o1(new_n180));
  oaoi03aa1n03x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d06x4               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n179), .o1(new_n184));
  oaoi03aa1n12x5               g089(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n185));
  nor042aa1n04x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nand42aa1n06x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  nanb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(new_n188));
  inv000aa1d42x5               g093(.a(new_n188), .o1(new_n189));
  aoai13aa1n06x5               g094(.a(new_n189), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n190));
  aoi112aa1n03x4               g095(.a(new_n189), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n191));
  norb02aa1n02x7               g096(.a(new_n190), .b(new_n191), .out0(\s[19] ));
  xnrc02aa1n02x5               g097(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  nanp02aa1n12x5               g099(.a(\b[19] ), .b(\a[20] ), .o1(new_n195));
  norb02aa1n02x5               g100(.a(new_n195), .b(new_n194), .out0(new_n196));
  nona22aa1n02x5               g101(.a(new_n190), .b(new_n196), .c(new_n186), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n196), .o1(new_n198));
  oaoi13aa1n06x5               g103(.a(new_n198), .b(new_n190), .c(\a[19] ), .d(\b[18] ), .o1(new_n199));
  norb02aa1n03x4               g104(.a(new_n197), .b(new_n199), .out0(\s[20] ));
  nona23aa1n09x5               g105(.a(new_n195), .b(new_n187), .c(new_n186), .d(new_n194), .out0(new_n201));
  norb02aa1n02x5               g106(.a(new_n183), .b(new_n201), .out0(new_n202));
  oai022aa1n02x5               g107(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n203));
  oaib12aa1n02x5               g108(.a(new_n203), .b(new_n178), .c(\b[17] ), .out0(new_n204));
  aoi012aa1n06x5               g109(.a(new_n194), .b(new_n186), .c(new_n195), .o1(new_n205));
  oai012aa1n12x5               g110(.a(new_n205), .b(new_n201), .c(new_n204), .o1(new_n206));
  xnrc02aa1n02x5               g111(.a(\b[20] ), .b(\a[21] ), .out0(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n206), .c(new_n176), .d(new_n202), .o1(new_n209));
  aoi112aa1n02x5               g114(.a(new_n208), .b(new_n206), .c(new_n176), .d(new_n202), .o1(new_n210));
  norb02aa1n02x7               g115(.a(new_n209), .b(new_n210), .out0(\s[21] ));
  nor042aa1n03x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[22] ), .b(\b[21] ), .out0(new_n213));
  nona22aa1n02x5               g118(.a(new_n209), .b(new_n213), .c(new_n212), .out0(new_n214));
  inv040aa1n03x5               g119(.a(new_n212), .o1(new_n215));
  aobi12aa1n06x5               g120(.a(new_n213), .b(new_n209), .c(new_n215), .out0(new_n216));
  norb02aa1n03x4               g121(.a(new_n214), .b(new_n216), .out0(\s[22] ));
  nano23aa1n09x5               g122(.a(new_n186), .b(new_n194), .c(new_n195), .d(new_n187), .out0(new_n218));
  inv000aa1d42x5               g123(.a(\a[21] ), .o1(new_n219));
  inv040aa1d32x5               g124(.a(\a[22] ), .o1(new_n220));
  xroi22aa1d06x4               g125(.a(new_n219), .b(\b[20] ), .c(new_n220), .d(\b[21] ), .out0(new_n221));
  inv030aa1n03x5               g126(.a(new_n221), .o1(new_n222));
  nano22aa1n03x7               g127(.a(new_n222), .b(new_n183), .c(new_n218), .out0(new_n223));
  inv040aa1n02x5               g128(.a(new_n205), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n221), .b(new_n224), .c(new_n218), .d(new_n185), .o1(new_n225));
  oao003aa1n02x5               g130(.a(\a[22] ), .b(\b[21] ), .c(new_n215), .carry(new_n226));
  nanp02aa1n02x5               g131(.a(new_n225), .b(new_n226), .o1(new_n227));
  xnrc02aa1n12x5               g132(.a(\b[22] ), .b(\a[23] ), .out0(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n227), .c(new_n176), .d(new_n223), .o1(new_n230));
  aoi112aa1n02x5               g135(.a(new_n229), .b(new_n227), .c(new_n176), .d(new_n223), .o1(new_n231));
  norb02aa1n02x7               g136(.a(new_n230), .b(new_n231), .out0(\s[23] ));
  nor042aa1n03x5               g137(.a(\b[22] ), .b(\a[23] ), .o1(new_n233));
  xorc02aa1n12x5               g138(.a(\a[24] ), .b(\b[23] ), .out0(new_n234));
  nona22aa1n02x5               g139(.a(new_n230), .b(new_n234), .c(new_n233), .out0(new_n235));
  inv000aa1n02x5               g140(.a(new_n233), .o1(new_n236));
  aobi12aa1n06x5               g141(.a(new_n234), .b(new_n230), .c(new_n236), .out0(new_n237));
  norb02aa1n03x4               g142(.a(new_n235), .b(new_n237), .out0(\s[24] ));
  nanb02aa1n06x5               g143(.a(new_n228), .b(new_n234), .out0(new_n239));
  nano32aa1n02x5               g144(.a(new_n239), .b(new_n221), .c(new_n183), .d(new_n218), .out0(new_n240));
  oao003aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .c(new_n236), .carry(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n239), .c(new_n225), .d(new_n226), .o1(new_n242));
  xorc02aa1n02x5               g147(.a(\a[25] ), .b(\b[24] ), .out0(new_n243));
  aoai13aa1n06x5               g148(.a(new_n243), .b(new_n242), .c(new_n176), .d(new_n240), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n243), .b(new_n242), .c(new_n176), .d(new_n240), .o1(new_n245));
  norb02aa1n02x7               g150(.a(new_n244), .b(new_n245), .out0(\s[25] ));
  nor042aa1n03x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n02x5               g152(.a(\a[26] ), .b(\b[25] ), .out0(new_n248));
  nona22aa1n02x5               g153(.a(new_n244), .b(new_n248), .c(new_n247), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n247), .o1(new_n250));
  aobi12aa1n06x5               g155(.a(new_n248), .b(new_n244), .c(new_n250), .out0(new_n251));
  norb02aa1n03x4               g156(.a(new_n249), .b(new_n251), .out0(\s[26] ));
  inv000aa1d42x5               g157(.a(\a[25] ), .o1(new_n253));
  inv040aa1d32x5               g158(.a(\a[26] ), .o1(new_n254));
  xroi22aa1d06x4               g159(.a(new_n253), .b(\b[24] ), .c(new_n254), .d(\b[25] ), .out0(new_n255));
  inv000aa1n02x5               g160(.a(new_n255), .o1(new_n256));
  nona22aa1n02x4               g161(.a(new_n223), .b(new_n239), .c(new_n256), .out0(new_n257));
  inv000aa1n02x5               g162(.a(new_n257), .o1(new_n258));
  nand22aa1n12x5               g163(.a(new_n176), .b(new_n258), .o1(new_n259));
  oao003aa1n02x5               g164(.a(\a[26] ), .b(\b[25] ), .c(new_n250), .carry(new_n260));
  aobi12aa1n09x5               g165(.a(new_n260), .b(new_n242), .c(new_n255), .out0(new_n261));
  xorc02aa1n12x5               g166(.a(\a[27] ), .b(\b[26] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n259), .out0(\s[27] ));
  norp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  inv040aa1n03x5               g169(.a(new_n264), .o1(new_n265));
  aobi12aa1n06x5               g170(.a(new_n262), .b(new_n261), .c(new_n259), .out0(new_n266));
  xnrc02aa1n02x5               g171(.a(\b[27] ), .b(\a[28] ), .out0(new_n267));
  nano22aa1n03x5               g172(.a(new_n266), .b(new_n265), .c(new_n267), .out0(new_n268));
  aoi013aa1n06x4               g173(.a(new_n257), .b(new_n173), .c(new_n174), .d(new_n175), .o1(new_n269));
  inv000aa1n02x5               g174(.a(new_n226), .o1(new_n270));
  inv000aa1n02x5               g175(.a(new_n239), .o1(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n206), .d(new_n221), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n260), .b(new_n256), .c(new_n272), .d(new_n241), .o1(new_n273));
  oaih12aa1n02x5               g178(.a(new_n262), .b(new_n273), .c(new_n269), .o1(new_n274));
  tech160nm_fiaoi012aa1n02p5x5 g179(.a(new_n267), .b(new_n274), .c(new_n265), .o1(new_n275));
  norp02aa1n03x5               g180(.a(new_n275), .b(new_n268), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n262), .b(new_n267), .out0(new_n277));
  oaih12aa1n02x5               g182(.a(new_n277), .b(new_n273), .c(new_n269), .o1(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n265), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n280), .b(new_n278), .c(new_n279), .o1(new_n281));
  aobi12aa1n06x5               g186(.a(new_n277), .b(new_n261), .c(new_n259), .out0(new_n282));
  nano22aa1n03x5               g187(.a(new_n282), .b(new_n279), .c(new_n280), .out0(new_n283));
  norp02aa1n03x5               g188(.a(new_n281), .b(new_n283), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n262), .b(new_n280), .c(new_n267), .out0(new_n286));
  oaih12aa1n02x5               g191(.a(new_n286), .b(new_n273), .c(new_n269), .o1(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  tech160nm_fiaoi012aa1n02p5x5 g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  aobi12aa1n06x5               g195(.a(new_n286), .b(new_n261), .c(new_n259), .out0(new_n291));
  nano22aa1n03x5               g196(.a(new_n291), .b(new_n288), .c(new_n289), .out0(new_n292));
  norp02aa1n03x5               g197(.a(new_n290), .b(new_n292), .o1(\s[30] ));
  xnrc02aa1n02x5               g198(.a(\b[30] ), .b(\a[31] ), .out0(new_n294));
  norb02aa1n03x4               g199(.a(new_n286), .b(new_n289), .out0(new_n295));
  oaih12aa1n02x5               g200(.a(new_n295), .b(new_n273), .c(new_n269), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n297));
  tech160nm_fiaoi012aa1n02p5x5 g202(.a(new_n294), .b(new_n296), .c(new_n297), .o1(new_n298));
  aobi12aa1n06x5               g203(.a(new_n295), .b(new_n261), .c(new_n259), .out0(new_n299));
  nano22aa1n03x5               g204(.a(new_n299), .b(new_n294), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[31] ));
  oai012aa1n02x5               g206(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g208(.a(\a[3] ), .b(\b[2] ), .c(new_n302), .o1(new_n304));
  xorb03aa1n02x5               g209(.a(new_n304), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g210(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fioai012aa1n05x5   g211(.a(new_n117), .b(new_n108), .c(new_n116), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  inv000aa1d42x5               g213(.a(new_n111), .o1(new_n309));
  oaib12aa1n03x5               g214(.a(new_n115), .b(new_n114), .c(new_n307), .out0(new_n310));
  xnbna2aa1n03x5               g215(.a(new_n310), .b(new_n309), .c(new_n112), .out0(\s[7] ));
  oaoi03aa1n03x5               g216(.a(\a[7] ), .b(\b[6] ), .c(new_n310), .o1(new_n312));
  xorb03aa1n02x5               g217(.a(new_n312), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  aoi112aa1n02x5               g218(.a(new_n122), .b(new_n124), .c(new_n119), .d(new_n108), .o1(new_n314));
  norb02aa1n02x5               g219(.a(new_n125), .b(new_n314), .out0(\s[9] ));
endmodule


