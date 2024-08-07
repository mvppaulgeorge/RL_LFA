// Benchmark "adder" written by ABC on Thu Jul 18 09:24:37 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n186, new_n187, new_n188,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n219, new_n220, new_n221,
    new_n222, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n244,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n292, new_n293,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n309, new_n312,
    new_n313, new_n315, new_n317, new_n318, new_n319, new_n320;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n02x5               g003(.a(\b[8] ), .b(\a[9] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor002aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nand22aa1n03x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  tech160nm_fioai012aa1n04x5   g007(.a(new_n100), .b(new_n101), .c(new_n102), .o1(new_n103));
  nand42aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n16x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norp02aa1n03x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nand42aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nona23aa1n09x5               g012(.a(new_n104), .b(new_n107), .c(new_n106), .d(new_n105), .out0(new_n108));
  aoi012aa1n02x5               g013(.a(new_n105), .b(new_n106), .c(new_n104), .o1(new_n109));
  oai012aa1n09x5               g014(.a(new_n109), .b(new_n108), .c(new_n103), .o1(new_n110));
  nor002aa1n12x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nand42aa1n04x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor022aa1n16x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n09x5               g019(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n115));
  xnrc02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[4] ), .b(\a[5] ), .out0(new_n117));
  nor043aa1n04x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  aoi112aa1n03x5               g025(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n121));
  aoi012aa1n02x5               g026(.a(new_n121), .b(new_n119), .c(new_n120), .o1(new_n122));
  aoi012aa1n02x5               g027(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n123));
  oai012aa1n12x5               g028(.a(new_n123), .b(new_n115), .c(new_n122), .o1(new_n124));
  tech160nm_fiao0012aa1n02p5x5 g029(.a(new_n124), .b(new_n110), .c(new_n118), .o(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n98), .b(new_n99), .c(new_n125), .d(new_n126), .o1(new_n127));
  aoai13aa1n02x5               g032(.a(new_n126), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n128));
  nona22aa1n02x4               g033(.a(new_n128), .b(new_n99), .c(new_n98), .out0(new_n129));
  nanp02aa1n02x5               g034(.a(new_n127), .b(new_n129), .o1(\s[10] ));
  nand42aa1n02x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nor022aa1n08x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1n04x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nanb02aa1n02x5               g038(.a(new_n132), .b(new_n133), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n129), .c(new_n131), .out0(\s[11] ));
  aoi013aa1n02x4               g040(.a(new_n132), .b(new_n129), .c(new_n131), .d(new_n133), .o1(new_n136));
  xnrb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  norp02aa1n12x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nand02aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nona23aa1d18x5               g044(.a(new_n139), .b(new_n133), .c(new_n132), .d(new_n138), .out0(new_n140));
  nano22aa1n02x4               g045(.a(new_n140), .b(new_n97), .c(new_n126), .out0(new_n141));
  aoai13aa1n02x5               g046(.a(new_n141), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n142));
  norp02aa1n02x5               g047(.a(\b[9] ), .b(\a[10] ), .o1(new_n143));
  tech160nm_fiaoi012aa1n05x5   g048(.a(new_n143), .b(new_n99), .c(new_n131), .o1(new_n144));
  aoi012aa1n06x5               g049(.a(new_n138), .b(new_n132), .c(new_n139), .o1(new_n145));
  oai012aa1n18x5               g050(.a(new_n145), .b(new_n140), .c(new_n144), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nor042aa1n04x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  nand42aa1n10x5               g053(.a(\b[12] ), .b(\a[13] ), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(new_n150));
  xobna2aa1n03x5               g055(.a(new_n150), .b(new_n142), .c(new_n147), .out0(\s[13] ));
  nanp02aa1n02x5               g056(.a(new_n142), .b(new_n147), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n148), .b(new_n152), .c(new_n149), .o1(new_n153));
  xnrb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand02aa1d16x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nanb02aa1n02x5               g061(.a(new_n155), .b(new_n156), .out0(new_n157));
  nona22aa1n02x4               g062(.a(new_n152), .b(new_n150), .c(new_n157), .out0(new_n158));
  aoi012aa1n12x5               g063(.a(new_n155), .b(new_n148), .c(new_n156), .o1(new_n159));
  nor042aa1n12x5               g064(.a(\b[14] ), .b(\a[15] ), .o1(new_n160));
  nand42aa1n03x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nanb02aa1n03x5               g066(.a(new_n160), .b(new_n161), .out0(new_n162));
  xobna2aa1n03x5               g067(.a(new_n162), .b(new_n158), .c(new_n159), .out0(\s[15] ));
  inv000aa1d42x5               g068(.a(new_n160), .o1(new_n164));
  aoi112aa1n02x5               g069(.a(new_n157), .b(new_n150), .c(new_n142), .d(new_n147), .o1(new_n165));
  inv000aa1d42x5               g070(.a(new_n159), .o1(new_n166));
  oabi12aa1n02x5               g071(.a(new_n162), .b(new_n165), .c(new_n166), .out0(new_n167));
  nor042aa1n02x5               g072(.a(\b[15] ), .b(\a[16] ), .o1(new_n168));
  nand42aa1n03x5               g073(.a(\b[15] ), .b(\a[16] ), .o1(new_n169));
  nanb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(new_n170));
  aoi012aa1n02x5               g075(.a(new_n170), .b(new_n167), .c(new_n164), .o1(new_n171));
  aoi012aa1n02x5               g076(.a(new_n162), .b(new_n158), .c(new_n159), .o1(new_n172));
  nano22aa1n02x4               g077(.a(new_n172), .b(new_n164), .c(new_n170), .out0(new_n173));
  norp02aa1n02x5               g078(.a(new_n171), .b(new_n173), .o1(\s[16] ));
  nano23aa1n06x5               g079(.a(new_n132), .b(new_n138), .c(new_n139), .d(new_n133), .out0(new_n175));
  nano23aa1n03x5               g080(.a(new_n160), .b(new_n168), .c(new_n169), .d(new_n161), .out0(new_n176));
  nano23aa1n02x4               g081(.a(new_n148), .b(new_n155), .c(new_n156), .d(new_n149), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n177), .b(new_n176), .o1(new_n178));
  nano32aa1n03x7               g083(.a(new_n178), .b(new_n175), .c(new_n126), .d(new_n97), .out0(new_n179));
  aoai13aa1n12x5               g084(.a(new_n179), .b(new_n124), .c(new_n110), .d(new_n118), .o1(new_n180));
  aoi012aa1n02x5               g085(.a(new_n168), .b(new_n160), .c(new_n169), .o1(new_n181));
  oai013aa1n06x5               g086(.a(new_n181), .b(new_n159), .c(new_n162), .d(new_n170), .o1(new_n182));
  aoib12aa1n12x5               g087(.a(new_n182), .b(new_n146), .c(new_n178), .out0(new_n183));
  xnrc02aa1n02x5               g088(.a(\b[16] ), .b(\a[17] ), .out0(new_n184));
  xobna2aa1n03x5               g089(.a(new_n184), .b(new_n180), .c(new_n183), .out0(\s[17] ));
  nanp02aa1n09x5               g090(.a(new_n180), .b(new_n183), .o1(new_n186));
  nor042aa1n02x5               g091(.a(\b[16] ), .b(\a[17] ), .o1(new_n187));
  aoib12aa1n03x5               g092(.a(new_n187), .b(new_n186), .c(new_n184), .out0(new_n188));
  xnrb03aa1n03x5               g093(.a(new_n188), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  nor042aa1n02x5               g094(.a(\b[17] ), .b(\a[18] ), .o1(new_n190));
  nand22aa1n04x5               g095(.a(\b[17] ), .b(\a[18] ), .o1(new_n191));
  nanb02aa1n02x5               g096(.a(new_n190), .b(new_n191), .out0(new_n192));
  aoi112aa1n03x5               g097(.a(new_n192), .b(new_n184), .c(new_n180), .d(new_n183), .o1(new_n193));
  tech160nm_fiaoi012aa1n04x5   g098(.a(new_n190), .b(new_n187), .c(new_n191), .o1(new_n194));
  inv000aa1n02x5               g099(.a(new_n194), .o1(new_n195));
  nor042aa1n04x5               g100(.a(new_n193), .b(new_n195), .o1(new_n196));
  nor022aa1n16x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(new_n197), .o1(new_n198));
  nand42aa1n03x5               g103(.a(\b[18] ), .b(\a[19] ), .o1(new_n199));
  xnbna2aa1n03x5               g104(.a(new_n196), .b(new_n199), .c(new_n198), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norb02aa1n02x5               g106(.a(new_n199), .b(new_n197), .out0(new_n202));
  tech160nm_fioai012aa1n04x5   g107(.a(new_n202), .b(new_n193), .c(new_n195), .o1(new_n203));
  nor002aa1n03x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand42aa1n04x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  tech160nm_fiaoi012aa1n05x5   g112(.a(new_n207), .b(new_n203), .c(new_n198), .o1(new_n208));
  nona22aa1n03x5               g113(.a(new_n203), .b(new_n206), .c(new_n197), .out0(new_n209));
  norb02aa1n03x4               g114(.a(new_n209), .b(new_n208), .out0(\s[20] ));
  nano23aa1n03x5               g115(.a(new_n197), .b(new_n204), .c(new_n205), .d(new_n199), .out0(new_n211));
  nona22aa1n02x4               g116(.a(new_n211), .b(new_n184), .c(new_n192), .out0(new_n212));
  nona23aa1n09x5               g117(.a(new_n205), .b(new_n199), .c(new_n197), .d(new_n204), .out0(new_n213));
  aoi012aa1n06x5               g118(.a(new_n204), .b(new_n197), .c(new_n205), .o1(new_n214));
  oai012aa1n12x5               g119(.a(new_n214), .b(new_n213), .c(new_n194), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n216), .b(new_n212), .c(new_n180), .d(new_n183), .o1(new_n217));
  xorb03aa1n02x5               g122(.a(new_n217), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  inv040aa1d32x5               g123(.a(\a[22] ), .o1(new_n219));
  inv000aa1d42x5               g124(.a(\a[21] ), .o1(new_n220));
  inv000aa1d42x5               g125(.a(\b[20] ), .o1(new_n221));
  oaoi03aa1n03x5               g126(.a(new_n220), .b(new_n221), .c(new_n217), .o1(new_n222));
  xorb03aa1n02x5               g127(.a(new_n222), .b(\b[21] ), .c(new_n219), .out0(\s[22] ));
  inv020aa1n03x5               g128(.a(new_n214), .o1(new_n224));
  xroi22aa1d06x4               g129(.a(new_n220), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n225));
  aoai13aa1n06x5               g130(.a(new_n225), .b(new_n224), .c(new_n211), .d(new_n195), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  norp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  oaoi03aa1n12x5               g133(.a(new_n219), .b(new_n227), .c(new_n228), .o1(new_n229));
  nand22aa1n03x5               g134(.a(new_n226), .b(new_n229), .o1(new_n230));
  inv000aa1n02x5               g135(.a(new_n230), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n225), .o1(new_n232));
  nona22aa1n02x4               g137(.a(new_n186), .b(new_n212), .c(new_n232), .out0(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[22] ), .b(\a[23] ), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  xnbna2aa1n03x5               g140(.a(new_n235), .b(new_n233), .c(new_n231), .out0(\s[23] ));
  nor042aa1n03x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  inv000aa1d42x5               g142(.a(new_n237), .o1(new_n238));
  aoi012aa1n06x5               g143(.a(new_n212), .b(new_n180), .c(new_n183), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n235), .b(new_n230), .c(new_n239), .d(new_n225), .o1(new_n240));
  xnrc02aa1n02x5               g145(.a(\b[23] ), .b(\a[24] ), .out0(new_n241));
  aoi012aa1n02x7               g146(.a(new_n241), .b(new_n240), .c(new_n238), .o1(new_n242));
  aoi012aa1n03x5               g147(.a(new_n234), .b(new_n233), .c(new_n231), .o1(new_n243));
  nano22aa1n03x5               g148(.a(new_n243), .b(new_n238), .c(new_n241), .out0(new_n244));
  nor002aa1n02x5               g149(.a(new_n242), .b(new_n244), .o1(\s[24] ));
  nor002aa1n02x5               g150(.a(new_n241), .b(new_n234), .o1(new_n246));
  inv030aa1n02x5               g151(.a(new_n246), .o1(new_n247));
  oao003aa1n02x5               g152(.a(\a[24] ), .b(\b[23] ), .c(new_n238), .carry(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n247), .c(new_n226), .d(new_n229), .o1(new_n249));
  inv000aa1n02x5               g154(.a(new_n249), .o1(new_n250));
  nona32aa1n02x4               g155(.a(new_n186), .b(new_n247), .c(new_n232), .d(new_n212), .out0(new_n251));
  xnrc02aa1n12x5               g156(.a(\b[24] ), .b(\a[25] ), .out0(new_n252));
  xobna2aa1n03x5               g157(.a(new_n252), .b(new_n251), .c(new_n250), .out0(\s[25] ));
  nor042aa1n03x5               g158(.a(\b[24] ), .b(\a[25] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  nanp02aa1n02x5               g160(.a(new_n225), .b(new_n246), .o1(new_n256));
  aoi112aa1n03x4               g161(.a(new_n256), .b(new_n212), .c(new_n180), .d(new_n183), .o1(new_n257));
  oabi12aa1n02x5               g162(.a(new_n252), .b(new_n257), .c(new_n249), .out0(new_n258));
  xnrc02aa1n02x5               g163(.a(\b[25] ), .b(\a[26] ), .out0(new_n259));
  tech160nm_fiaoi012aa1n02p5x5 g164(.a(new_n259), .b(new_n258), .c(new_n255), .o1(new_n260));
  tech160nm_fiaoi012aa1n05x5   g165(.a(new_n252), .b(new_n251), .c(new_n250), .o1(new_n261));
  nano22aa1n03x7               g166(.a(new_n261), .b(new_n255), .c(new_n259), .out0(new_n262));
  norp02aa1n03x5               g167(.a(new_n260), .b(new_n262), .o1(\s[26] ));
  xorc02aa1n02x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  norp02aa1n02x5               g169(.a(new_n259), .b(new_n252), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n265), .o1(new_n266));
  nona32aa1n09x5               g171(.a(new_n186), .b(new_n266), .c(new_n256), .d(new_n212), .out0(new_n267));
  oao003aa1n02x5               g172(.a(\a[26] ), .b(\b[25] ), .c(new_n255), .carry(new_n268));
  aobi12aa1n12x5               g173(.a(new_n268), .b(new_n249), .c(new_n265), .out0(new_n269));
  xnbna2aa1n03x5               g174(.a(new_n264), .b(new_n269), .c(new_n267), .out0(\s[27] ));
  norp02aa1n02x5               g175(.a(\b[26] ), .b(\a[27] ), .o1(new_n271));
  inv040aa1n03x5               g176(.a(new_n271), .o1(new_n272));
  nano22aa1n02x4               g177(.a(new_n232), .b(new_n246), .c(new_n265), .out0(new_n273));
  inv000aa1d42x5               g178(.a(new_n229), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n246), .b(new_n274), .c(new_n215), .d(new_n225), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n268), .b(new_n266), .c(new_n275), .d(new_n248), .o1(new_n276));
  aoai13aa1n03x5               g181(.a(new_n264), .b(new_n276), .c(new_n239), .d(new_n273), .o1(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .out0(new_n278));
  tech160nm_fiaoi012aa1n02p5x5 g183(.a(new_n278), .b(new_n277), .c(new_n272), .o1(new_n279));
  aobi12aa1n02x7               g184(.a(new_n264), .b(new_n269), .c(new_n267), .out0(new_n280));
  nano22aa1n03x5               g185(.a(new_n280), .b(new_n272), .c(new_n278), .out0(new_n281));
  norp02aa1n03x5               g186(.a(new_n279), .b(new_n281), .o1(\s[28] ));
  xnrc02aa1n02x5               g187(.a(\b[28] ), .b(\a[29] ), .out0(new_n283));
  norb02aa1n02x5               g188(.a(new_n264), .b(new_n278), .out0(new_n284));
  aoai13aa1n03x5               g189(.a(new_n284), .b(new_n276), .c(new_n239), .d(new_n273), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[28] ), .b(\b[27] ), .c(new_n272), .carry(new_n286));
  tech160nm_fiaoi012aa1n02p5x5 g191(.a(new_n283), .b(new_n285), .c(new_n286), .o1(new_n287));
  aobi12aa1n06x5               g192(.a(new_n284), .b(new_n269), .c(new_n267), .out0(new_n288));
  nano22aa1n03x5               g193(.a(new_n288), .b(new_n283), .c(new_n286), .out0(new_n289));
  norp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[29] ));
  xorb03aa1n02x5               g195(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  xnrc02aa1n02x5               g196(.a(\b[29] ), .b(\a[30] ), .out0(new_n292));
  norb03aa1n02x5               g197(.a(new_n264), .b(new_n283), .c(new_n278), .out0(new_n293));
  aoai13aa1n03x5               g198(.a(new_n293), .b(new_n276), .c(new_n239), .d(new_n273), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[29] ), .b(\b[28] ), .c(new_n286), .carry(new_n295));
  tech160nm_fiaoi012aa1n02p5x5 g200(.a(new_n292), .b(new_n294), .c(new_n295), .o1(new_n296));
  aobi12aa1n06x5               g201(.a(new_n293), .b(new_n269), .c(new_n267), .out0(new_n297));
  nano22aa1n03x5               g202(.a(new_n297), .b(new_n292), .c(new_n295), .out0(new_n298));
  norp02aa1n03x5               g203(.a(new_n296), .b(new_n298), .o1(\s[30] ));
  norb02aa1n02x5               g204(.a(new_n293), .b(new_n292), .out0(new_n300));
  aobi12aa1n02x7               g205(.a(new_n300), .b(new_n269), .c(new_n267), .out0(new_n301));
  oao003aa1n02x5               g206(.a(\a[30] ), .b(\b[29] ), .c(new_n295), .carry(new_n302));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  nano22aa1n03x5               g208(.a(new_n301), .b(new_n302), .c(new_n303), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n300), .b(new_n276), .c(new_n239), .d(new_n273), .o1(new_n305));
  tech160nm_fiaoi012aa1n02p5x5 g210(.a(new_n303), .b(new_n305), .c(new_n302), .o1(new_n306));
  norp02aa1n03x5               g211(.a(new_n306), .b(new_n304), .o1(\s[31] ));
  xnrb03aa1n02x5               g212(.a(new_n103), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g213(.a(\a[3] ), .b(\b[2] ), .c(new_n103), .o1(new_n309));
  xorb03aa1n02x5               g214(.a(new_n309), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g215(.a(new_n110), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g216(.a(new_n117), .b(new_n110), .out0(new_n312));
  oai012aa1n02x5               g217(.a(new_n312), .b(\b[4] ), .c(\a[5] ), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oao003aa1n02x5               g219(.a(new_n119), .b(new_n120), .c(new_n313), .carry(new_n315));
  xorb03aa1n02x5               g220(.a(new_n315), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nanb02aa1n02x5               g221(.a(new_n111), .b(new_n112), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n317), .o1(new_n318));
  aoai13aa1n02x5               g223(.a(new_n318), .b(new_n113), .c(new_n315), .d(new_n114), .o1(new_n319));
  aoi112aa1n02x5               g224(.a(new_n318), .b(new_n113), .c(new_n315), .d(new_n114), .o1(new_n320));
  norb02aa1n02x5               g225(.a(new_n319), .b(new_n320), .out0(\s[8] ));
  xorb03aa1n02x5               g226(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


