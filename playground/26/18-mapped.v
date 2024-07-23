// Benchmark "adder" written by ABC on Thu Jul 18 01:25:41 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n128, new_n129, new_n130, new_n131, new_n133,
    new_n135, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n148, new_n149,
    new_n151, new_n152, new_n153, new_n154, new_n155, new_n156, new_n158,
    new_n159, new_n160, new_n161, new_n162, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n174,
    new_n175, new_n176, new_n177, new_n179, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n196, new_n197, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n206, new_n207,
    new_n208, new_n209, new_n210, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n222, new_n223,
    new_n224, new_n225, new_n226, new_n228, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n240, new_n241, new_n242, new_n243, new_n244, new_n246, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n266, new_n267, new_n268, new_n269, new_n270,
    new_n271, new_n272, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n292, new_n295, new_n296, new_n298, new_n300;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n03x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n09x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nor002aa1d32x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nand02aa1n04x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1d18x5               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  nor043aa1n04x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor042aa1n09x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nand22aa1n04x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nand02aa1n04x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  aoi012aa1n12x5               g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nand22aa1n12x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nand42aa1n03x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  ao0012aa1n03x7               g019(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n115));
  oabi12aa1n18x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .out0(new_n116));
  nanp02aa1n02x5               g021(.a(new_n100), .b(new_n99), .o1(new_n117));
  oaih22aa1d12x5               g022(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n118));
  aob012aa1n12x5               g023(.a(new_n118), .b(\b[5] ), .c(\a[6] ), .out0(new_n119));
  oai122aa1n12x5               g024(.a(new_n117), .b(new_n102), .c(new_n119), .d(\b[7] ), .e(\a[8] ), .o1(new_n120));
  xorc02aa1n02x5               g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoai13aa1n02x5               g026(.a(new_n121), .b(new_n120), .c(new_n105), .d(new_n116), .o1(new_n122));
  tech160nm_fixnrc02aa1n04x5   g027(.a(\b[9] ), .b(\a[10] ), .out0(new_n123));
  xobna2aa1n03x5               g028(.a(new_n123), .b(new_n122), .c(new_n97), .out0(\s[10] ));
  norp02aa1n02x5               g029(.a(new_n102), .b(new_n119), .o1(new_n125));
  aoi112aa1n02x7               g030(.a(new_n125), .b(new_n98), .c(new_n99), .d(new_n100), .o1(new_n126));
  aob012aa1n06x5               g031(.a(new_n126), .b(new_n116), .c(new_n105), .out0(new_n127));
  tech160nm_fioaoi03aa1n03p5x5 g032(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n128));
  nanp02aa1n02x5               g033(.a(\b[8] ), .b(\a[9] ), .o1(new_n129));
  nano22aa1n06x5               g034(.a(new_n123), .b(new_n97), .c(new_n129), .out0(new_n130));
  aoi012aa1n03x5               g035(.a(new_n128), .b(new_n127), .c(new_n130), .o1(new_n131));
  xnrb03aa1n02x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g037(.a(\a[11] ), .b(\b[10] ), .c(new_n131), .o1(new_n133));
  xorb03aa1n02x5               g038(.a(new_n133), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n02x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  nand22aa1n02x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nor042aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand22aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nano23aa1n06x5               g043(.a(new_n135), .b(new_n137), .c(new_n138), .d(new_n136), .out0(new_n139));
  nano22aa1n02x4               g044(.a(new_n123), .b(new_n139), .c(new_n121), .out0(new_n140));
  aoai13aa1n06x5               g045(.a(new_n140), .b(new_n120), .c(new_n105), .d(new_n116), .o1(new_n141));
  tech160nm_fiao0012aa1n02p5x5 g046(.a(new_n137), .b(new_n135), .c(new_n138), .o(new_n142));
  aoi012aa1n06x5               g047(.a(new_n142), .b(new_n139), .c(new_n128), .o1(new_n143));
  nor002aa1n20x5               g048(.a(\b[12] ), .b(\a[13] ), .o1(new_n144));
  nand42aa1d28x5               g049(.a(\b[12] ), .b(\a[13] ), .o1(new_n145));
  nanb02aa1n02x5               g050(.a(new_n144), .b(new_n145), .out0(new_n146));
  xobna2aa1n03x5               g051(.a(new_n146), .b(new_n141), .c(new_n143), .out0(\s[13] ));
  inv000aa1d42x5               g052(.a(new_n144), .o1(new_n148));
  aoai13aa1n02x5               g053(.a(new_n148), .b(new_n146), .c(new_n141), .d(new_n143), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g055(.a(\b[13] ), .b(\a[14] ), .o1(new_n151));
  nand42aa1n16x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nano23aa1d15x5               g057(.a(new_n144), .b(new_n151), .c(new_n152), .d(new_n145), .out0(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  tech160nm_fioai012aa1n04x5   g059(.a(new_n152), .b(new_n151), .c(new_n144), .o1(new_n155));
  aoai13aa1n04x5               g060(.a(new_n155), .b(new_n154), .c(new_n141), .d(new_n143), .o1(new_n156));
  xorb03aa1n02x5               g061(.a(new_n156), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g062(.a(\b[14] ), .b(\a[15] ), .o1(new_n158));
  xorc02aa1n12x5               g063(.a(\a[15] ), .b(\b[14] ), .out0(new_n159));
  xorc02aa1n12x5               g064(.a(\a[16] ), .b(\b[15] ), .out0(new_n160));
  aoi112aa1n02x5               g065(.a(new_n160), .b(new_n158), .c(new_n156), .d(new_n159), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n160), .b(new_n158), .c(new_n156), .d(new_n159), .o1(new_n162));
  norb02aa1n02x7               g067(.a(new_n162), .b(new_n161), .out0(\s[16] ));
  nand23aa1d12x5               g068(.a(new_n153), .b(new_n159), .c(new_n160), .o1(new_n164));
  nano22aa1d15x5               g069(.a(new_n164), .b(new_n130), .c(new_n139), .out0(new_n165));
  aoai13aa1n12x5               g070(.a(new_n165), .b(new_n120), .c(new_n105), .d(new_n116), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n160), .b(new_n159), .o1(new_n167));
  orn002aa1n02x5               g072(.a(\a[15] ), .b(\b[14] ), .o(new_n168));
  oao003aa1n02x5               g073(.a(\a[16] ), .b(\b[15] ), .c(new_n168), .carry(new_n169));
  tech160nm_fioai012aa1n04x5   g074(.a(new_n169), .b(new_n167), .c(new_n155), .o1(new_n170));
  oab012aa1d15x5               g075(.a(new_n170), .b(new_n143), .c(new_n164), .out0(new_n171));
  nanp02aa1n09x5               g076(.a(new_n166), .b(new_n171), .o1(new_n172));
  xorb03aa1n02x5               g077(.a(new_n172), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g078(.a(\a[18] ), .o1(new_n174));
  inv000aa1d42x5               g079(.a(\a[17] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\b[16] ), .o1(new_n176));
  oaoi03aa1n02x5               g081(.a(new_n175), .b(new_n176), .c(new_n172), .o1(new_n177));
  xorb03aa1n02x5               g082(.a(new_n177), .b(\b[17] ), .c(new_n174), .out0(\s[18] ));
  xroi22aa1d04x5               g083(.a(new_n175), .b(\b[16] ), .c(new_n174), .d(\b[17] ), .out0(new_n179));
  nanp02aa1n02x5               g084(.a(new_n176), .b(new_n175), .o1(new_n180));
  oaoi03aa1n02x5               g085(.a(\a[18] ), .b(\b[17] ), .c(new_n180), .o1(new_n181));
  nor002aa1n20x5               g086(.a(\b[18] ), .b(\a[19] ), .o1(new_n182));
  nand02aa1d28x5               g087(.a(\b[18] ), .b(\a[19] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n181), .c(new_n172), .d(new_n179), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n181), .c(new_n172), .d(new_n179), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(\s[19] ));
  xnrc02aa1n02x5               g092(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n06x5               g093(.a(\b[19] ), .b(\a[20] ), .o1(new_n189));
  nand02aa1d12x5               g094(.a(\b[19] ), .b(\a[20] ), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  nona22aa1n02x5               g096(.a(new_n185), .b(new_n191), .c(new_n182), .out0(new_n192));
  inv000aa1d42x5               g097(.a(new_n191), .o1(new_n193));
  oaoi13aa1n06x5               g098(.a(new_n193), .b(new_n185), .c(\a[19] ), .d(\b[18] ), .o1(new_n194));
  norb02aa1n03x4               g099(.a(new_n192), .b(new_n194), .out0(\s[20] ));
  nano23aa1n06x5               g100(.a(new_n182), .b(new_n189), .c(new_n190), .d(new_n183), .out0(new_n196));
  nanp02aa1n02x5               g101(.a(new_n179), .b(new_n196), .o1(new_n197));
  oai022aa1n02x5               g102(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n198));
  oaib12aa1n06x5               g103(.a(new_n198), .b(new_n174), .c(\b[17] ), .out0(new_n199));
  nona23aa1n09x5               g104(.a(new_n190), .b(new_n183), .c(new_n182), .d(new_n189), .out0(new_n200));
  aoi012aa1n09x5               g105(.a(new_n189), .b(new_n182), .c(new_n190), .o1(new_n201));
  oai012aa1n18x5               g106(.a(new_n201), .b(new_n200), .c(new_n199), .o1(new_n202));
  inv000aa1d42x5               g107(.a(new_n202), .o1(new_n203));
  aoai13aa1n04x5               g108(.a(new_n203), .b(new_n197), .c(new_n166), .d(new_n171), .o1(new_n204));
  xorb03aa1n02x5               g109(.a(new_n204), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g110(.a(\b[20] ), .b(\a[21] ), .o1(new_n206));
  xorc02aa1n02x5               g111(.a(\a[21] ), .b(\b[20] ), .out0(new_n207));
  xorc02aa1n02x5               g112(.a(\a[22] ), .b(\b[21] ), .out0(new_n208));
  aoi112aa1n02x7               g113(.a(new_n206), .b(new_n208), .c(new_n204), .d(new_n207), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n208), .b(new_n206), .c(new_n204), .d(new_n207), .o1(new_n210));
  norb02aa1n02x7               g115(.a(new_n210), .b(new_n209), .out0(\s[22] ));
  inv000aa1d42x5               g116(.a(\a[21] ), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\a[22] ), .o1(new_n213));
  xroi22aa1d04x5               g118(.a(new_n212), .b(\b[20] ), .c(new_n213), .d(\b[21] ), .out0(new_n214));
  nanp03aa1n02x5               g119(.a(new_n214), .b(new_n179), .c(new_n196), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[21] ), .o1(new_n216));
  oaoi03aa1n12x5               g121(.a(new_n213), .b(new_n216), .c(new_n206), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  aoi012aa1n02x5               g123(.a(new_n218), .b(new_n202), .c(new_n214), .o1(new_n219));
  aoai13aa1n04x5               g124(.a(new_n219), .b(new_n215), .c(new_n166), .d(new_n171), .o1(new_n220));
  xorb03aa1n02x5               g125(.a(new_n220), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g126(.a(\b[22] ), .b(\a[23] ), .o1(new_n222));
  tech160nm_fixorc02aa1n02p5x5 g127(.a(\a[23] ), .b(\b[22] ), .out0(new_n223));
  xorc02aa1n02x5               g128(.a(\a[24] ), .b(\b[23] ), .out0(new_n224));
  aoi112aa1n02x5               g129(.a(new_n222), .b(new_n224), .c(new_n220), .d(new_n223), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n224), .b(new_n222), .c(new_n220), .d(new_n223), .o1(new_n226));
  norb02aa1n02x7               g131(.a(new_n226), .b(new_n225), .out0(\s[24] ));
  and002aa1n06x5               g132(.a(new_n224), .b(new_n223), .o(new_n228));
  inv000aa1n02x5               g133(.a(new_n228), .o1(new_n229));
  nano32aa1n02x4               g134(.a(new_n229), .b(new_n214), .c(new_n179), .d(new_n196), .out0(new_n230));
  inv040aa1n03x5               g135(.a(new_n201), .o1(new_n231));
  aoai13aa1n03x5               g136(.a(new_n214), .b(new_n231), .c(new_n196), .d(new_n181), .o1(new_n232));
  orn002aa1n02x5               g137(.a(\a[23] ), .b(\b[22] ), .o(new_n233));
  oao003aa1n02x5               g138(.a(\a[24] ), .b(\b[23] ), .c(new_n233), .carry(new_n234));
  aoai13aa1n06x5               g139(.a(new_n234), .b(new_n229), .c(new_n232), .d(new_n217), .o1(new_n235));
  xorc02aa1n12x5               g140(.a(\a[25] ), .b(\b[24] ), .out0(new_n236));
  aoai13aa1n06x5               g141(.a(new_n236), .b(new_n235), .c(new_n172), .d(new_n230), .o1(new_n237));
  aoi112aa1n02x5               g142(.a(new_n236), .b(new_n235), .c(new_n172), .d(new_n230), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n237), .b(new_n238), .out0(\s[25] ));
  nor042aa1n03x5               g144(.a(\b[24] ), .b(\a[25] ), .o1(new_n240));
  xorc02aa1n12x5               g145(.a(\a[26] ), .b(\b[25] ), .out0(new_n241));
  nona22aa1n02x5               g146(.a(new_n237), .b(new_n241), .c(new_n240), .out0(new_n242));
  inv000aa1d42x5               g147(.a(new_n240), .o1(new_n243));
  aobi12aa1n06x5               g148(.a(new_n241), .b(new_n237), .c(new_n243), .out0(new_n244));
  norb02aa1n03x4               g149(.a(new_n242), .b(new_n244), .out0(\s[26] ));
  oai122aa1n02x7               g150(.a(new_n169), .b(new_n143), .c(new_n164), .d(new_n167), .e(new_n155), .o1(new_n246));
  and002aa1n06x5               g151(.a(new_n241), .b(new_n236), .o(new_n247));
  nano22aa1n03x7               g152(.a(new_n215), .b(new_n228), .c(new_n247), .out0(new_n248));
  aoai13aa1n06x5               g153(.a(new_n248), .b(new_n246), .c(new_n127), .d(new_n165), .o1(new_n249));
  oao003aa1n02x5               g154(.a(\a[26] ), .b(\b[25] ), .c(new_n243), .carry(new_n250));
  aobi12aa1n06x5               g155(.a(new_n250), .b(new_n235), .c(new_n247), .out0(new_n251));
  xorc02aa1n02x5               g156(.a(\a[27] ), .b(\b[26] ), .out0(new_n252));
  xnbna2aa1n06x5               g157(.a(new_n252), .b(new_n251), .c(new_n249), .out0(\s[27] ));
  norp02aa1n02x5               g158(.a(\b[26] ), .b(\a[27] ), .o1(new_n254));
  inv040aa1n03x5               g159(.a(new_n254), .o1(new_n255));
  aobi12aa1n06x5               g160(.a(new_n252), .b(new_n251), .c(new_n249), .out0(new_n256));
  xnrc02aa1n02x5               g161(.a(\b[27] ), .b(\a[28] ), .out0(new_n257));
  nano22aa1n03x5               g162(.a(new_n256), .b(new_n255), .c(new_n257), .out0(new_n258));
  aobi12aa1n06x5               g163(.a(new_n248), .b(new_n166), .c(new_n171), .out0(new_n259));
  aoai13aa1n04x5               g164(.a(new_n228), .b(new_n218), .c(new_n202), .d(new_n214), .o1(new_n260));
  inv000aa1d42x5               g165(.a(new_n247), .o1(new_n261));
  aoai13aa1n06x5               g166(.a(new_n250), .b(new_n261), .c(new_n260), .d(new_n234), .o1(new_n262));
  oaih12aa1n02x5               g167(.a(new_n252), .b(new_n262), .c(new_n259), .o1(new_n263));
  tech160nm_fiaoi012aa1n02p5x5 g168(.a(new_n257), .b(new_n263), .c(new_n255), .o1(new_n264));
  norp02aa1n03x5               g169(.a(new_n264), .b(new_n258), .o1(\s[28] ));
  norb02aa1n02x5               g170(.a(new_n252), .b(new_n257), .out0(new_n266));
  aobi12aa1n06x5               g171(.a(new_n266), .b(new_n251), .c(new_n249), .out0(new_n267));
  oao003aa1n02x5               g172(.a(\a[28] ), .b(\b[27] ), .c(new_n255), .carry(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[28] ), .b(\a[29] ), .out0(new_n269));
  nano22aa1n03x5               g174(.a(new_n267), .b(new_n268), .c(new_n269), .out0(new_n270));
  tech160nm_fioai012aa1n03p5x5 g175(.a(new_n266), .b(new_n262), .c(new_n259), .o1(new_n271));
  tech160nm_fiaoi012aa1n04x5   g176(.a(new_n269), .b(new_n271), .c(new_n268), .o1(new_n272));
  norp02aa1n03x5               g177(.a(new_n272), .b(new_n270), .o1(\s[29] ));
  xorb03aa1n02x5               g178(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g179(.a(new_n252), .b(new_n269), .c(new_n257), .out0(new_n275));
  aobi12aa1n06x5               g180(.a(new_n275), .b(new_n251), .c(new_n249), .out0(new_n276));
  oao003aa1n02x5               g181(.a(\a[29] ), .b(\b[28] ), .c(new_n268), .carry(new_n277));
  xnrc02aa1n02x5               g182(.a(\b[29] ), .b(\a[30] ), .out0(new_n278));
  nano22aa1n03x5               g183(.a(new_n276), .b(new_n277), .c(new_n278), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n275), .b(new_n262), .c(new_n259), .o1(new_n280));
  tech160nm_fiaoi012aa1n02p5x5 g185(.a(new_n278), .b(new_n280), .c(new_n277), .o1(new_n281));
  norp02aa1n03x5               g186(.a(new_n281), .b(new_n279), .o1(\s[30] ));
  xnrc02aa1n02x5               g187(.a(\b[30] ), .b(\a[31] ), .out0(new_n283));
  norb02aa1n02x5               g188(.a(new_n275), .b(new_n278), .out0(new_n284));
  aobi12aa1n06x5               g189(.a(new_n284), .b(new_n251), .c(new_n249), .out0(new_n285));
  oao003aa1n02x5               g190(.a(\a[30] ), .b(\b[29] ), .c(new_n277), .carry(new_n286));
  nano22aa1n03x5               g191(.a(new_n285), .b(new_n283), .c(new_n286), .out0(new_n287));
  oaih12aa1n02x5               g192(.a(new_n284), .b(new_n262), .c(new_n259), .o1(new_n288));
  tech160nm_fiaoi012aa1n02p5x5 g193(.a(new_n283), .b(new_n288), .c(new_n286), .o1(new_n289));
  norp02aa1n03x5               g194(.a(new_n289), .b(new_n287), .o1(\s[31] ));
  xnrb03aa1n02x5               g195(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g196(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n292));
  xorb03aa1n02x5               g197(.a(new_n292), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g198(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  inv000aa1d42x5               g199(.a(new_n116), .o1(new_n295));
  oaoi03aa1n02x5               g200(.a(\a[5] ), .b(\b[4] ), .c(new_n295), .o1(new_n296));
  xorb03aa1n02x5               g201(.a(new_n296), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g202(.a(new_n119), .b(new_n295), .c(new_n103), .d(new_n104), .o1(new_n298));
  xorb03aa1n02x5               g203(.a(new_n298), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g204(.a(new_n100), .b(new_n298), .c(new_n101), .o1(new_n300));
  xnrb03aa1n02x5               g205(.a(new_n300), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g206(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


