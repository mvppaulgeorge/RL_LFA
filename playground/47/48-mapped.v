// Benchmark "adder" written by ABC on Thu Jul 18 12:32:30 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n220,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n227, new_n228,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n277, new_n278, new_n279, new_n280, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n289, new_n290,
    new_n291, new_n292, new_n294, new_n295, new_n296, new_n297, new_n298,
    new_n299, new_n300, new_n302, new_n304, new_n305, new_n306, new_n307,
    new_n308, new_n309, new_n310, new_n312, new_n313, new_n314, new_n315,
    new_n316, new_n317, new_n318, new_n321, new_n324, new_n326, new_n328;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  tech160nm_finand02aa1n05x5   g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n06x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  orn002aa1n02x5               g004(.a(\a[9] ), .b(\b[8] ), .o(new_n100));
  nor022aa1n16x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  inv000aa1d42x5               g006(.a(new_n101), .o1(new_n102));
  nand02aa1n08x5               g007(.a(\b[7] ), .b(\a[8] ), .o1(new_n103));
  inv000aa1d42x5               g008(.a(new_n103), .o1(new_n104));
  nor002aa1n02x5               g009(.a(\b[6] ), .b(\a[7] ), .o1(new_n105));
  inv000aa1n02x5               g010(.a(new_n105), .o1(new_n106));
  oai022aa1n03x5               g011(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[6] ), .b(\a[7] ), .o1(new_n108));
  nand02aa1n06x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nanp03aa1n02x5               g014(.a(new_n107), .b(new_n108), .c(new_n109), .o1(new_n110));
  aoai13aa1n09x5               g015(.a(new_n102), .b(new_n104), .c(new_n110), .d(new_n106), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor022aa1n08x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor022aa1n04x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  oai012aa1n02x5               g019(.a(new_n112), .b(new_n114), .c(new_n113), .o1(new_n115));
  inv000aa1d42x5               g020(.a(\a[1] ), .o1(new_n116));
  inv040aa1d32x5               g021(.a(\b[0] ), .o1(new_n117));
  norp02aa1n04x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  nanp02aa1n02x5               g023(.a(\b[1] ), .b(\a[2] ), .o1(new_n119));
  oaoi13aa1n09x5               g024(.a(new_n118), .b(new_n119), .c(new_n116), .d(new_n117), .o1(new_n120));
  nanp02aa1n02x5               g025(.a(\b[2] ), .b(\a[3] ), .o1(new_n121));
  nona23aa1n09x5               g026(.a(new_n112), .b(new_n121), .c(new_n114), .d(new_n113), .out0(new_n122));
  oaih12aa1n06x5               g027(.a(new_n115), .b(new_n122), .c(new_n120), .o1(new_n123));
  orn002aa1n02x5               g028(.a(\a[5] ), .b(\b[4] ), .o(new_n124));
  nanp02aa1n02x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  nona23aa1n02x4               g030(.a(new_n108), .b(new_n103), .c(new_n101), .d(new_n105), .out0(new_n126));
  nor002aa1n02x5               g031(.a(\b[5] ), .b(\a[6] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n109), .b(new_n127), .out0(new_n128));
  nano32aa1n03x7               g033(.a(new_n126), .b(new_n128), .c(new_n124), .d(new_n125), .out0(new_n129));
  nor002aa1n03x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nand42aa1n03x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norb02aa1n03x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  aoai13aa1n02x5               g037(.a(new_n132), .b(new_n111), .c(new_n123), .d(new_n129), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n99), .b(new_n133), .c(new_n100), .out0(\s[10] ));
  tech160nm_fixorc02aa1n04x5   g039(.a(\a[11] ), .b(\b[10] ), .out0(new_n135));
  nona22aa1n02x4               g040(.a(new_n133), .b(new_n130), .c(new_n97), .out0(new_n136));
  xobna2aa1n03x5               g041(.a(new_n135), .b(new_n136), .c(new_n98), .out0(\s[11] ));
  nor002aa1n03x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n09x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  norb02aa1n03x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  nor042aa1n02x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  aoi013aa1n02x4               g046(.a(new_n141), .b(new_n136), .c(new_n135), .d(new_n98), .o1(new_n142));
  xnrc02aa1n02x5               g047(.a(new_n142), .b(new_n140), .out0(\s[12] ));
  nand23aa1n03x5               g048(.a(new_n132), .b(new_n99), .c(new_n140), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n135), .b(new_n144), .out0(new_n145));
  aoai13aa1n04x5               g050(.a(new_n145), .b(new_n111), .c(new_n123), .d(new_n129), .o1(new_n146));
  norb03aa1n03x5               g051(.a(new_n139), .b(new_n141), .c(new_n138), .out0(new_n147));
  aoi022aa1d24x5               g052(.a(\b[9] ), .b(\a[10] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n148));
  tech160nm_fioai012aa1n05x5   g053(.a(new_n148), .b(new_n130), .c(new_n97), .o1(new_n149));
  aob012aa1n02x5               g054(.a(new_n139), .b(new_n147), .c(new_n149), .out0(new_n150));
  nor002aa1d24x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nand42aa1n06x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  norb02aa1n02x5               g057(.a(new_n152), .b(new_n151), .out0(new_n153));
  xnbna2aa1n03x5               g058(.a(new_n153), .b(new_n146), .c(new_n150), .out0(\s[13] ));
  inv000aa1d42x5               g059(.a(new_n151), .o1(new_n155));
  inv020aa1n03x5               g060(.a(new_n152), .o1(new_n156));
  aoai13aa1n02x5               g061(.a(new_n155), .b(new_n156), .c(new_n146), .d(new_n150), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n12x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  nanp02aa1n04x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nano23aa1n02x4               g065(.a(new_n151), .b(new_n159), .c(new_n160), .d(new_n152), .out0(new_n161));
  inv000aa1n02x5               g066(.a(new_n161), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n155), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n06x5               g069(.a(new_n164), .b(new_n162), .c(new_n146), .d(new_n150), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n04x5               g071(.a(\b[14] ), .b(\a[15] ), .o1(new_n167));
  tech160nm_finand02aa1n03p5x5 g072(.a(\b[14] ), .b(\a[15] ), .o1(new_n168));
  nor042aa1n04x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanp02aa1n04x5               g074(.a(\b[15] ), .b(\a[16] ), .o1(new_n170));
  norb02aa1n06x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoi112aa1n02x5               g076(.a(new_n167), .b(new_n171), .c(new_n165), .d(new_n168), .o1(new_n172));
  aoai13aa1n03x5               g077(.a(new_n171), .b(new_n167), .c(new_n165), .d(new_n168), .o1(new_n173));
  norb02aa1n02x7               g078(.a(new_n173), .b(new_n172), .out0(\s[16] ));
  norb02aa1n02x7               g079(.a(new_n160), .b(new_n159), .out0(new_n175));
  nona32aa1n03x5               g080(.a(new_n175), .b(new_n156), .c(new_n167), .d(new_n151), .out0(new_n176));
  nano22aa1n02x4               g081(.a(new_n169), .b(new_n168), .c(new_n170), .out0(new_n177));
  nano23aa1n06x5               g082(.a(new_n144), .b(new_n176), .c(new_n177), .d(new_n135), .out0(new_n178));
  aoai13aa1n12x5               g083(.a(new_n178), .b(new_n111), .c(new_n123), .d(new_n129), .o1(new_n179));
  oaoi13aa1n02x5               g084(.a(new_n167), .b(new_n160), .c(new_n151), .d(new_n159), .o1(new_n180));
  nano22aa1n03x5               g085(.a(new_n180), .b(new_n168), .c(new_n170), .out0(new_n181));
  oai012aa1n02x5               g086(.a(new_n139), .b(\b[12] ), .c(\a[13] ), .o1(new_n182));
  aoi012aa1n03x5               g087(.a(new_n182), .b(new_n147), .c(new_n149), .o1(new_n183));
  nona23aa1n02x4               g088(.a(new_n160), .b(new_n152), .c(new_n167), .d(new_n159), .out0(new_n184));
  nano22aa1n02x5               g089(.a(new_n184), .b(new_n171), .c(new_n168), .out0(new_n185));
  aoi112aa1n09x5               g090(.a(new_n169), .b(new_n181), .c(new_n185), .d(new_n183), .o1(new_n186));
  xnrc02aa1n12x5               g091(.a(\b[16] ), .b(\a[17] ), .out0(new_n187));
  xobna2aa1n03x5               g092(.a(new_n187), .b(new_n179), .c(new_n186), .out0(\s[17] ));
  inv000aa1d42x5               g093(.a(\a[18] ), .o1(new_n189));
  nanp02aa1n06x5               g094(.a(new_n179), .b(new_n186), .o1(new_n190));
  norp02aa1n02x5               g095(.a(\b[16] ), .b(\a[17] ), .o1(new_n191));
  aoib12aa1n06x5               g096(.a(new_n191), .b(new_n190), .c(new_n187), .out0(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xnrc02aa1n02x5               g098(.a(\b[17] ), .b(\a[18] ), .out0(new_n194));
  nor042aa1n04x5               g099(.a(new_n194), .b(new_n187), .o1(new_n195));
  inv000aa1d42x5               g100(.a(new_n195), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[17] ), .b(\a[18] ), .o1(new_n197));
  oai022aa1n04x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  and002aa1n02x5               g103(.a(new_n198), .b(new_n197), .o(new_n199));
  inv000aa1d42x5               g104(.a(new_n199), .o1(new_n200));
  aoai13aa1n04x5               g105(.a(new_n200), .b(new_n196), .c(new_n179), .d(new_n186), .o1(new_n201));
  xorb03aa1n02x5               g106(.a(new_n201), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1n04x5               g108(.a(\b[18] ), .b(\a[19] ), .o1(new_n204));
  nand02aa1d04x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nor042aa1n03x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  nand02aa1n06x5               g111(.a(\b[19] ), .b(\a[20] ), .o1(new_n207));
  norb02aa1n02x5               g112(.a(new_n207), .b(new_n206), .out0(new_n208));
  aoi112aa1n02x5               g113(.a(new_n204), .b(new_n208), .c(new_n201), .d(new_n205), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n208), .b(new_n204), .c(new_n201), .d(new_n205), .o1(new_n210));
  norb02aa1n03x4               g115(.a(new_n210), .b(new_n209), .out0(\s[20] ));
  nanb03aa1d18x5               g116(.a(new_n206), .b(new_n207), .c(new_n205), .out0(new_n212));
  nona22aa1n02x4               g117(.a(new_n195), .b(new_n204), .c(new_n212), .out0(new_n213));
  inv000aa1d42x5               g118(.a(\b[18] ), .o1(new_n214));
  nanb02aa1n02x5               g119(.a(\a[19] ), .b(new_n214), .out0(new_n215));
  nand23aa1n03x5               g120(.a(new_n198), .b(new_n215), .c(new_n197), .o1(new_n216));
  aoi012aa1n09x5               g121(.a(new_n206), .b(new_n204), .c(new_n207), .o1(new_n217));
  oai012aa1d24x5               g122(.a(new_n217), .b(new_n216), .c(new_n212), .o1(new_n218));
  inv000aa1d42x5               g123(.a(new_n218), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n213), .c(new_n179), .d(new_n186), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g126(.a(\b[20] ), .b(\a[21] ), .o1(new_n222));
  xnrc02aa1n12x5               g127(.a(\b[20] ), .b(\a[21] ), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  xnrc02aa1n12x5               g129(.a(\b[21] ), .b(\a[22] ), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoi112aa1n02x5               g131(.a(new_n222), .b(new_n226), .c(new_n220), .d(new_n224), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n226), .b(new_n222), .c(new_n220), .d(new_n224), .o1(new_n228));
  norb02aa1n03x4               g133(.a(new_n228), .b(new_n227), .out0(\s[22] ));
  nor042aa1n06x5               g134(.a(new_n225), .b(new_n223), .o1(new_n230));
  nona23aa1n09x5               g135(.a(new_n195), .b(new_n230), .c(new_n212), .d(new_n204), .out0(new_n231));
  inv000aa1d42x5               g136(.a(\a[22] ), .o1(new_n232));
  inv000aa1d42x5               g137(.a(\b[21] ), .o1(new_n233));
  oaoi03aa1n09x5               g138(.a(new_n232), .b(new_n233), .c(new_n222), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoi012aa1n02x5               g140(.a(new_n235), .b(new_n218), .c(new_n230), .o1(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n231), .c(new_n179), .d(new_n186), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g143(.a(\b[22] ), .b(\a[23] ), .o1(new_n239));
  xorc02aa1n12x5               g144(.a(\a[23] ), .b(\b[22] ), .out0(new_n240));
  xorc02aa1n02x5               g145(.a(\a[24] ), .b(\b[23] ), .out0(new_n241));
  aoi112aa1n03x5               g146(.a(new_n239), .b(new_n241), .c(new_n237), .d(new_n240), .o1(new_n242));
  aoai13aa1n03x5               g147(.a(new_n241), .b(new_n239), .c(new_n237), .d(new_n240), .o1(new_n243));
  norb02aa1n03x4               g148(.a(new_n243), .b(new_n242), .out0(\s[24] ));
  nano32aa1n06x5               g149(.a(new_n213), .b(new_n241), .c(new_n230), .d(new_n240), .out0(new_n245));
  nano22aa1n02x4               g150(.a(new_n206), .b(new_n205), .c(new_n207), .out0(new_n246));
  oai012aa1n02x5               g151(.a(new_n197), .b(\b[18] ), .c(\a[19] ), .o1(new_n247));
  norb02aa1n02x5               g152(.a(new_n198), .b(new_n247), .out0(new_n248));
  inv000aa1n02x5               g153(.a(new_n217), .o1(new_n249));
  aoai13aa1n06x5               g154(.a(new_n230), .b(new_n249), .c(new_n248), .d(new_n246), .o1(new_n250));
  and002aa1n02x5               g155(.a(new_n241), .b(new_n240), .o(new_n251));
  inv000aa1n02x5               g156(.a(new_n251), .o1(new_n252));
  orn002aa1n02x5               g157(.a(\a[23] ), .b(\b[22] ), .o(new_n253));
  oao003aa1n02x5               g158(.a(\a[24] ), .b(\b[23] ), .c(new_n253), .carry(new_n254));
  aoai13aa1n06x5               g159(.a(new_n254), .b(new_n252), .c(new_n250), .d(new_n234), .o1(new_n255));
  xorc02aa1n12x5               g160(.a(\a[25] ), .b(\b[24] ), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n255), .c(new_n190), .d(new_n245), .o1(new_n257));
  aoi112aa1n02x5               g162(.a(new_n256), .b(new_n255), .c(new_n190), .d(new_n245), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n257), .b(new_n258), .out0(\s[25] ));
  nor042aa1n03x5               g164(.a(\b[24] ), .b(\a[25] ), .o1(new_n260));
  xorc02aa1n03x5               g165(.a(\a[26] ), .b(\b[25] ), .out0(new_n261));
  nona22aa1n02x5               g166(.a(new_n257), .b(new_n261), .c(new_n260), .out0(new_n262));
  inv000aa1d42x5               g167(.a(new_n260), .o1(new_n263));
  aobi12aa1n06x5               g168(.a(new_n261), .b(new_n257), .c(new_n263), .out0(new_n264));
  norb02aa1n03x4               g169(.a(new_n262), .b(new_n264), .out0(\s[26] ));
  nanp02aa1n02x5               g170(.a(new_n110), .b(new_n106), .o1(new_n266));
  aoi012aa1n02x5               g171(.a(new_n101), .b(new_n266), .c(new_n103), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(new_n123), .b(new_n129), .o1(new_n268));
  nanp02aa1n02x5               g173(.a(new_n268), .b(new_n267), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(new_n185), .b(new_n183), .o1(new_n270));
  nona22aa1n02x4               g175(.a(new_n270), .b(new_n181), .c(new_n169), .out0(new_n271));
  and002aa1n06x5               g176(.a(new_n261), .b(new_n256), .o(new_n272));
  nano22aa1n03x7               g177(.a(new_n231), .b(new_n251), .c(new_n272), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n271), .c(new_n269), .d(new_n178), .o1(new_n274));
  oao003aa1n02x5               g179(.a(\a[26] ), .b(\b[25] ), .c(new_n263), .carry(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  tech160nm_fiaoi012aa1n05x5   g181(.a(new_n276), .b(new_n255), .c(new_n272), .o1(new_n277));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[26] ), .b(\a[27] ), .o1(new_n279));
  norb02aa1n02x5               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  xnbna2aa1n03x5               g185(.a(new_n280), .b(new_n277), .c(new_n274), .out0(\s[27] ));
  inv000aa1n06x5               g186(.a(new_n278), .o1(new_n282));
  aobi12aa1n02x7               g187(.a(new_n280), .b(new_n277), .c(new_n274), .out0(new_n283));
  xnrc02aa1n02x5               g188(.a(\b[27] ), .b(\a[28] ), .out0(new_n284));
  nano22aa1n02x4               g189(.a(new_n283), .b(new_n282), .c(new_n284), .out0(new_n285));
  inv020aa1n03x5               g190(.a(new_n273), .o1(new_n286));
  aoi012aa1n06x5               g191(.a(new_n286), .b(new_n179), .c(new_n186), .o1(new_n287));
  aoai13aa1n06x5               g192(.a(new_n251), .b(new_n235), .c(new_n218), .d(new_n230), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n272), .o1(new_n289));
  aoai13aa1n06x5               g194(.a(new_n275), .b(new_n289), .c(new_n288), .d(new_n254), .o1(new_n290));
  oaih12aa1n02x5               g195(.a(new_n280), .b(new_n290), .c(new_n287), .o1(new_n291));
  aoi012aa1n03x5               g196(.a(new_n284), .b(new_n291), .c(new_n282), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n285), .o1(\s[28] ));
  nano22aa1n02x4               g198(.a(new_n284), .b(new_n282), .c(new_n279), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n287), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[28] ), .b(\b[27] ), .c(new_n282), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[28] ), .b(\a[29] ), .out0(new_n297));
  aoi012aa1n03x5               g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  aobi12aa1n02x7               g203(.a(new_n294), .b(new_n277), .c(new_n274), .out0(new_n299));
  nano22aa1n02x4               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  norp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[29] ));
  and002aa1n02x5               g206(.a(\b[0] ), .b(\a[1] ), .o(new_n302));
  xnrb03aa1n02x5               g207(.a(new_n302), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano23aa1n02x4               g208(.a(new_n297), .b(new_n284), .c(new_n279), .d(new_n282), .out0(new_n304));
  oaih12aa1n02x5               g209(.a(new_n304), .b(new_n290), .c(new_n287), .o1(new_n305));
  oao003aa1n02x5               g210(.a(\a[29] ), .b(\b[28] ), .c(new_n296), .carry(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[29] ), .b(\a[30] ), .out0(new_n307));
  aoi012aa1n03x5               g212(.a(new_n307), .b(new_n305), .c(new_n306), .o1(new_n308));
  aobi12aa1n02x7               g213(.a(new_n304), .b(new_n277), .c(new_n274), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n306), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[30] ));
  norb03aa1n02x5               g216(.a(new_n294), .b(new_n307), .c(new_n297), .out0(new_n312));
  aobi12aa1n06x5               g217(.a(new_n312), .b(new_n277), .c(new_n274), .out0(new_n313));
  oao003aa1n02x5               g218(.a(\a[30] ), .b(\b[29] ), .c(new_n306), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[30] ), .b(\a[31] ), .out0(new_n315));
  nano22aa1n03x5               g220(.a(new_n313), .b(new_n314), .c(new_n315), .out0(new_n316));
  oaih12aa1n02x5               g221(.a(new_n312), .b(new_n290), .c(new_n287), .o1(new_n317));
  aoi012aa1n03x5               g222(.a(new_n315), .b(new_n317), .c(new_n314), .o1(new_n318));
  norp02aa1n03x5               g223(.a(new_n318), .b(new_n316), .o1(\s[31] ));
  xnrb03aa1n02x5               g224(.a(new_n120), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g225(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g227(.a(new_n123), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aob012aa1n02x5               g228(.a(new_n124), .b(new_n123), .c(new_n125), .out0(new_n324));
  xorb03aa1n02x5               g229(.a(new_n324), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fioai012aa1n05x5   g230(.a(new_n109), .b(new_n324), .c(new_n127), .o1(new_n326));
  xnbna2aa1n03x5               g231(.a(new_n326), .b(new_n106), .c(new_n108), .out0(\s[7] ));
  oaoi03aa1n02x5               g232(.a(\a[7] ), .b(\b[6] ), .c(new_n326), .o1(new_n328));
  xorb03aa1n02x5               g233(.a(new_n328), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g234(.a(new_n132), .b(new_n268), .c(new_n267), .out0(\s[9] ));
endmodule


