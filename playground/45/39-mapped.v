// Benchmark "adder" written by ABC on Thu Jul 18 11:24:27 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n154, new_n156, new_n157,
    new_n158, new_n159, new_n160, new_n161, new_n162, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n216, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n232, new_n233, new_n234, new_n235, new_n236, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n257, new_n258, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n266, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n282, new_n283, new_n284,
    new_n285, new_n286, new_n287, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n295, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n306, new_n309, new_n311, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  xorc02aa1n12x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand22aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  tech160nm_fioaoi03aa1n03p5x5 g006(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n102));
  nor022aa1n04x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nand22aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nor022aa1n08x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nona23aa1n03x5               g011(.a(new_n106), .b(new_n104), .c(new_n103), .d(new_n105), .out0(new_n107));
  ao0012aa1n03x7               g012(.a(new_n103), .b(new_n105), .c(new_n104), .o(new_n108));
  oabi12aa1n09x5               g013(.a(new_n108), .b(new_n107), .c(new_n102), .out0(new_n109));
  nor002aa1n02x5               g014(.a(\b[6] ), .b(\a[7] ), .o1(new_n110));
  nand22aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanp02aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nona23aa1n09x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  nand02aa1d06x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nor042aa1d18x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb02aa1n03x5               g021(.a(new_n116), .b(new_n115), .out0(new_n117));
  xorc02aa1n02x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  norb03aa1n03x5               g023(.a(new_n118), .b(new_n114), .c(new_n117), .out0(new_n119));
  inv000aa1n02x5               g024(.a(new_n110), .o1(new_n120));
  orn002aa1n02x5               g025(.a(\a[8] ), .b(\b[7] ), .o(new_n121));
  and002aa1n02x5               g026(.a(\b[7] ), .b(\a[8] ), .o(new_n122));
  aoai13aa1n06x5               g027(.a(new_n111), .b(new_n116), .c(new_n112), .d(new_n115), .o1(new_n123));
  aoai13aa1n06x5               g028(.a(new_n121), .b(new_n122), .c(new_n123), .d(new_n120), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n119), .d(new_n109), .o1(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n97), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  nanp02aa1n02x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nand42aa1d28x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nor002aa1n04x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  oai022aa1d18x5               g036(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n132));
  nanb02aa1n03x5               g037(.a(new_n132), .b(new_n126), .out0(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n131), .b(new_n133), .c(new_n128), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(\a[12] ), .o1(new_n135));
  aoi013aa1n03x5               g040(.a(new_n130), .b(new_n133), .c(new_n128), .d(new_n129), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(new_n135), .out0(\s[12] ));
  oao003aa1n02x5               g042(.a(new_n99), .b(new_n100), .c(new_n101), .carry(new_n138));
  nano23aa1n02x4               g043(.a(new_n103), .b(new_n105), .c(new_n106), .d(new_n104), .out0(new_n139));
  aoi012aa1n02x7               g044(.a(new_n108), .b(new_n139), .c(new_n138), .o1(new_n140));
  inv030aa1n02x5               g045(.a(new_n117), .o1(new_n141));
  nanb03aa1n03x5               g046(.a(new_n114), .b(new_n118), .c(new_n141), .out0(new_n142));
  oabi12aa1n06x5               g047(.a(new_n124), .b(new_n142), .c(new_n140), .out0(new_n143));
  nor042aa1n02x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  tech160nm_finand02aa1n05x5   g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  nano23aa1n06x5               g050(.a(new_n144), .b(new_n130), .c(new_n145), .d(new_n129), .out0(new_n146));
  nand23aa1d12x5               g051(.a(new_n146), .b(new_n97), .c(new_n125), .o1(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  aoi022aa1n02x5               g053(.a(\b[11] ), .b(\a[12] ), .c(\a[11] ), .d(\b[10] ), .o1(new_n149));
  aoai13aa1n03x5               g054(.a(new_n149), .b(new_n130), .c(new_n132), .d(new_n128), .o1(new_n150));
  oaib12aa1n09x5               g055(.a(new_n150), .b(\b[11] ), .c(new_n135), .out0(new_n151));
  tech160nm_fiaoi012aa1n05x5   g056(.a(new_n151), .b(new_n143), .c(new_n148), .o1(new_n152));
  xnrb03aa1n02x5               g057(.a(new_n152), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  oaoi03aa1n03x5               g058(.a(\a[13] ), .b(\b[12] ), .c(new_n152), .o1(new_n154));
  xorb03aa1n02x5               g059(.a(new_n154), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  tech160nm_fixnrc02aa1n02p5x5 g060(.a(\b[12] ), .b(\a[13] ), .out0(new_n156));
  xnrc02aa1n02x5               g061(.a(\b[13] ), .b(\a[14] ), .out0(new_n157));
  orn002aa1n02x5               g062(.a(new_n156), .b(new_n157), .o(new_n158));
  norp02aa1n02x5               g063(.a(\b[13] ), .b(\a[14] ), .o1(new_n159));
  aoi112aa1n03x5               g064(.a(\b[12] ), .b(\a[13] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n160));
  norp02aa1n02x5               g065(.a(new_n160), .b(new_n159), .o1(new_n161));
  tech160nm_fioai012aa1n05x5   g066(.a(new_n161), .b(new_n152), .c(new_n158), .o1(new_n162));
  xorb03aa1n02x5               g067(.a(new_n162), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g068(.a(\b[14] ), .b(\a[15] ), .o1(new_n164));
  nand02aa1n06x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nor042aa1n02x5               g070(.a(\b[15] ), .b(\a[16] ), .o1(new_n166));
  nand02aa1n10x5               g071(.a(\b[15] ), .b(\a[16] ), .o1(new_n167));
  nanb02aa1n02x5               g072(.a(new_n166), .b(new_n167), .out0(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n164), .c(new_n162), .d(new_n165), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n165), .b(new_n164), .out0(new_n170));
  nanp02aa1n02x5               g075(.a(new_n162), .b(new_n170), .o1(new_n171));
  nona22aa1n02x4               g076(.a(new_n171), .b(new_n168), .c(new_n164), .out0(new_n172));
  nanp02aa1n03x5               g077(.a(new_n172), .b(new_n169), .o1(\s[16] ));
  nano23aa1n06x5               g078(.a(new_n164), .b(new_n166), .c(new_n167), .d(new_n165), .out0(new_n174));
  nona22aa1n09x5               g079(.a(new_n174), .b(new_n157), .c(new_n156), .out0(new_n175));
  nor042aa1n06x5               g080(.a(new_n175), .b(new_n147), .o1(new_n176));
  aoai13aa1n12x5               g081(.a(new_n176), .b(new_n124), .c(new_n109), .d(new_n119), .o1(new_n177));
  oai012aa1n02x5               g082(.a(new_n165), .b(new_n160), .c(new_n159), .o1(new_n178));
  nona22aa1n03x5               g083(.a(new_n178), .b(new_n166), .c(new_n164), .out0(new_n179));
  aboi22aa1n12x5               g084(.a(new_n175), .b(new_n151), .c(new_n179), .d(new_n167), .out0(new_n180));
  nanp02aa1n06x5               g085(.a(new_n177), .b(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d04x5               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nand22aa1n09x5               g093(.a(\b[17] ), .b(\a[18] ), .o1(new_n189));
  nona22aa1n03x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(new_n190));
  oaib12aa1n09x5               g095(.a(new_n190), .b(\b[17] ), .c(new_n183), .out0(new_n191));
  nor022aa1n16x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nand02aa1d06x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n191), .c(new_n181), .d(new_n188), .o1(new_n196));
  norb02aa1n02x7               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n12x5               g103(.a(\b[19] ), .b(\a[20] ), .o1(new_n199));
  nand02aa1d08x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanb02aa1n02x5               g105(.a(new_n199), .b(new_n200), .out0(new_n201));
  tech160nm_fioai012aa1n03p5x5 g106(.a(new_n195), .b(\b[18] ), .c(\a[19] ), .o1(new_n202));
  nanp02aa1n03x5               g107(.a(new_n202), .b(new_n201), .o1(new_n203));
  nona22aa1n02x5               g108(.a(new_n195), .b(new_n201), .c(new_n192), .out0(new_n204));
  nanp02aa1n03x5               g109(.a(new_n203), .b(new_n204), .o1(\s[20] ));
  nano23aa1n06x5               g110(.a(new_n192), .b(new_n199), .c(new_n200), .d(new_n193), .out0(new_n206));
  nanp02aa1n02x5               g111(.a(new_n188), .b(new_n206), .o1(new_n207));
  norp02aa1n02x5               g112(.a(\b[17] ), .b(\a[18] ), .o1(new_n208));
  aoi013aa1n06x5               g113(.a(new_n208), .b(new_n189), .c(new_n184), .d(new_n185), .o1(new_n209));
  nona23aa1n09x5               g114(.a(new_n200), .b(new_n193), .c(new_n192), .d(new_n199), .out0(new_n210));
  tech160nm_fioai012aa1n05x5   g115(.a(new_n200), .b(new_n199), .c(new_n192), .o1(new_n211));
  oai012aa1n12x5               g116(.a(new_n211), .b(new_n210), .c(new_n209), .o1(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n04x5               g118(.a(new_n213), .b(new_n207), .c(new_n177), .d(new_n180), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g120(.a(\b[20] ), .b(\a[21] ), .o1(new_n216));
  xnrc02aa1n12x5               g121(.a(\b[20] ), .b(\a[21] ), .out0(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  tech160nm_fixnrc02aa1n04x5   g123(.a(\b[21] ), .b(\a[22] ), .out0(new_n219));
  aoai13aa1n02x5               g124(.a(new_n219), .b(new_n216), .c(new_n214), .d(new_n218), .o1(new_n220));
  aoi112aa1n03x4               g125(.a(new_n216), .b(new_n219), .c(new_n214), .d(new_n218), .o1(new_n221));
  nanb02aa1n03x5               g126(.a(new_n221), .b(new_n220), .out0(\s[22] ));
  nor042aa1n06x5               g127(.a(new_n219), .b(new_n217), .o1(new_n223));
  nanp03aa1n02x5               g128(.a(new_n188), .b(new_n223), .c(new_n206), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\a[22] ), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\b[21] ), .o1(new_n226));
  oaoi03aa1n12x5               g131(.a(new_n225), .b(new_n226), .c(new_n216), .o1(new_n227));
  inv000aa1d42x5               g132(.a(new_n227), .o1(new_n228));
  aoi012aa1n02x5               g133(.a(new_n228), .b(new_n212), .c(new_n223), .o1(new_n229));
  aoai13aa1n04x5               g134(.a(new_n229), .b(new_n224), .c(new_n177), .d(new_n180), .o1(new_n230));
  xorb03aa1n02x5               g135(.a(new_n230), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g136(.a(\b[22] ), .b(\a[23] ), .o1(new_n232));
  xorc02aa1n12x5               g137(.a(\a[23] ), .b(\b[22] ), .out0(new_n233));
  xnrc02aa1n12x5               g138(.a(\b[23] ), .b(\a[24] ), .out0(new_n234));
  aoai13aa1n03x5               g139(.a(new_n234), .b(new_n232), .c(new_n230), .d(new_n233), .o1(new_n235));
  aoi112aa1n03x4               g140(.a(new_n232), .b(new_n234), .c(new_n230), .d(new_n233), .o1(new_n236));
  nanb02aa1n03x5               g141(.a(new_n236), .b(new_n235), .out0(\s[24] ));
  nanp02aa1n02x5               g142(.a(new_n179), .b(new_n167), .o1(new_n238));
  oaib12aa1n02x5               g143(.a(new_n238), .b(new_n175), .c(new_n151), .out0(new_n239));
  norb02aa1n02x7               g144(.a(new_n233), .b(new_n234), .out0(new_n240));
  inv000aa1n02x5               g145(.a(new_n240), .o1(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n188), .c(new_n223), .d(new_n206), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n239), .c(new_n143), .d(new_n176), .o1(new_n243));
  inv000aa1n02x5               g148(.a(new_n211), .o1(new_n244));
  aoai13aa1n06x5               g149(.a(new_n223), .b(new_n244), .c(new_n206), .d(new_n191), .o1(new_n245));
  orn002aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .o(new_n246));
  oao003aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .c(new_n246), .carry(new_n247));
  aoai13aa1n12x5               g152(.a(new_n247), .b(new_n241), .c(new_n245), .d(new_n227), .o1(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  tech160nm_fixorc02aa1n05x5   g154(.a(\a[25] ), .b(\b[24] ), .out0(new_n250));
  xnbna2aa1n03x5               g155(.a(new_n250), .b(new_n243), .c(new_n249), .out0(\s[25] ));
  nanp02aa1n02x5               g156(.a(new_n243), .b(new_n249), .o1(new_n252));
  norp02aa1n02x5               g157(.a(\b[24] ), .b(\a[25] ), .o1(new_n253));
  xorc02aa1n12x5               g158(.a(\a[26] ), .b(\b[25] ), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n255), .b(new_n253), .c(new_n252), .d(new_n250), .o1(new_n256));
  aoai13aa1n03x5               g161(.a(new_n250), .b(new_n248), .c(new_n181), .d(new_n242), .o1(new_n257));
  nona22aa1n03x5               g162(.a(new_n257), .b(new_n255), .c(new_n253), .out0(new_n258));
  nanp02aa1n02x5               g163(.a(new_n256), .b(new_n258), .o1(\s[26] ));
  and002aa1n18x5               g164(.a(new_n254), .b(new_n250), .o(new_n260));
  nano22aa1n03x7               g165(.a(new_n224), .b(new_n260), .c(new_n240), .out0(new_n261));
  aoai13aa1n06x5               g166(.a(new_n261), .b(new_n239), .c(new_n143), .d(new_n176), .o1(new_n262));
  aoi112aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n263));
  oab012aa1n02x4               g168(.a(new_n263), .b(\a[26] ), .c(\b[25] ), .out0(new_n264));
  aobi12aa1n09x5               g169(.a(new_n264), .b(new_n248), .c(new_n260), .out0(new_n265));
  xorc02aa1n02x5               g170(.a(\a[27] ), .b(\b[26] ), .out0(new_n266));
  xnbna2aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n262), .out0(\s[27] ));
  nand02aa1n03x5               g172(.a(new_n265), .b(new_n262), .o1(new_n268));
  norp02aa1n02x5               g173(.a(\b[26] ), .b(\a[27] ), .o1(new_n269));
  norp02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .o1(new_n270));
  nand42aa1n03x5               g175(.a(\b[27] ), .b(\a[28] ), .o1(new_n271));
  norb02aa1n02x5               g176(.a(new_n271), .b(new_n270), .out0(new_n272));
  inv040aa1n03x5               g177(.a(new_n272), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n269), .c(new_n268), .d(new_n266), .o1(new_n274));
  aobi12aa1n06x5               g179(.a(new_n261), .b(new_n177), .c(new_n180), .out0(new_n275));
  aoai13aa1n03x5               g180(.a(new_n240), .b(new_n228), .c(new_n212), .d(new_n223), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n260), .o1(new_n277));
  aoai13aa1n06x5               g182(.a(new_n264), .b(new_n277), .c(new_n276), .d(new_n247), .o1(new_n278));
  oai012aa1n02x5               g183(.a(new_n266), .b(new_n278), .c(new_n275), .o1(new_n279));
  nona22aa1n02x4               g184(.a(new_n279), .b(new_n273), .c(new_n269), .out0(new_n280));
  nanp02aa1n03x5               g185(.a(new_n274), .b(new_n280), .o1(\s[28] ));
  norb02aa1n02x5               g186(.a(new_n266), .b(new_n273), .out0(new_n282));
  oaih12aa1n02x5               g187(.a(new_n282), .b(new_n278), .c(new_n275), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .out0(new_n284));
  oai012aa1n02x5               g189(.a(new_n271), .b(new_n270), .c(new_n269), .o1(new_n285));
  norb02aa1n02x5               g190(.a(new_n285), .b(new_n284), .out0(new_n286));
  nanp02aa1n03x5               g191(.a(new_n283), .b(new_n285), .o1(new_n287));
  aoi022aa1n02x7               g192(.a(new_n287), .b(new_n284), .c(new_n283), .d(new_n286), .o1(\s[29] ));
  xorb03aa1n02x5               g193(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g194(.a(new_n266), .b(new_n284), .c(new_n272), .o(new_n290));
  oaih12aa1n02x5               g195(.a(new_n290), .b(new_n278), .c(new_n275), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[30] ), .b(\b[29] ), .out0(new_n292));
  oao003aa1n02x5               g197(.a(\a[29] ), .b(\b[28] ), .c(new_n285), .carry(new_n293));
  norb02aa1n02x5               g198(.a(new_n293), .b(new_n292), .out0(new_n294));
  nanp02aa1n03x5               g199(.a(new_n291), .b(new_n293), .o1(new_n295));
  aoi022aa1n02x7               g200(.a(new_n295), .b(new_n292), .c(new_n291), .d(new_n294), .o1(\s[30] ));
  and003aa1n02x5               g201(.a(new_n282), .b(new_n292), .c(new_n284), .o(new_n297));
  oai012aa1n02x5               g202(.a(new_n297), .b(new_n278), .c(new_n275), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[30] ), .b(\b[29] ), .c(new_n293), .carry(new_n299));
  xnrc02aa1n02x5               g204(.a(\b[30] ), .b(\a[31] ), .out0(new_n300));
  aoi012aa1n02x5               g205(.a(new_n300), .b(new_n298), .c(new_n299), .o1(new_n301));
  aobi12aa1n03x5               g206(.a(new_n297), .b(new_n265), .c(new_n262), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n302), .b(new_n299), .c(new_n300), .out0(new_n303));
  norp02aa1n03x5               g208(.a(new_n301), .b(new_n303), .o1(\s[31] ));
  xnrb03aa1n02x5               g209(.a(new_n102), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g210(.a(\a[3] ), .b(\b[2] ), .c(new_n102), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g212(.a(new_n109), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiaoi012aa1n05x5   g213(.a(new_n112), .b(new_n109), .c(new_n113), .o1(new_n309));
  xnrc02aa1n02x5               g214(.a(new_n309), .b(new_n141), .out0(\s[6] ));
  aob012aa1n02x5               g215(.a(new_n115), .b(new_n309), .c(new_n141), .out0(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n311), .b(new_n120), .c(new_n111), .out0(\s[7] ));
  oaoi03aa1n03x5               g217(.a(\a[7] ), .b(\b[6] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g219(.a(new_n143), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


