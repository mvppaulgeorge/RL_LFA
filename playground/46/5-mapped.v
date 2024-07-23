// Benchmark "adder" written by ABC on Thu Jul 18 11:34:27 2024

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
    new_n134, new_n135, new_n136, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n150, new_n151, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n161, new_n162, new_n163, new_n164, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n175, new_n177, new_n178, new_n179, new_n180, new_n181, new_n182,
    new_n185, new_n186, new_n187, new_n188, new_n189, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n199, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n209, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n215, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230,
    new_n231, new_n232, new_n233, new_n234, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n252, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n300, new_n302, new_n304, new_n306,
    new_n308;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor042aa1n04x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nor002aa1n03x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  nand22aa1n09x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nor002aa1n04x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nand22aa1n03x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nano23aa1n06x5               g007(.a(new_n99), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n103));
  nor002aa1n03x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nand22aa1n09x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor042aa1n06x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  ao0012aa1n03x7               g011(.a(new_n104), .b(new_n106), .c(new_n105), .o(new_n107));
  tech160nm_fiao0012aa1n02p5x5 g012(.a(new_n99), .b(new_n101), .c(new_n100), .o(new_n108));
  aoi012aa1n12x5               g013(.a(new_n108), .b(new_n103), .c(new_n107), .o1(new_n109));
  xorc02aa1n02x5               g014(.a(\a[4] ), .b(\b[3] ), .out0(new_n110));
  nor042aa1n02x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nand42aa1n03x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  norb02aa1n02x7               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  and002aa1n06x5               g018(.a(\b[1] ), .b(\a[2] ), .o(new_n114));
  nand02aa1d06x5               g019(.a(\b[0] ), .b(\a[1] ), .o1(new_n115));
  nor042aa1n02x5               g020(.a(\b[1] ), .b(\a[2] ), .o1(new_n116));
  oab012aa1n06x5               g021(.a(new_n114), .b(new_n116), .c(new_n115), .out0(new_n117));
  nanp03aa1n06x5               g022(.a(new_n117), .b(new_n110), .c(new_n113), .o1(new_n118));
  aoi112aa1n02x5               g023(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n119));
  oab012aa1n04x5               g024(.a(new_n119), .b(\a[4] ), .c(\b[3] ), .out0(new_n120));
  nand42aa1n03x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  nano23aa1n02x5               g026(.a(new_n104), .b(new_n106), .c(new_n121), .d(new_n105), .out0(new_n122));
  nand42aa1n02x5               g027(.a(new_n122), .b(new_n103), .o1(new_n123));
  aoai13aa1n12x5               g028(.a(new_n109), .b(new_n123), .c(new_n118), .d(new_n120), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoi012aa1n02x5               g030(.a(new_n98), .b(new_n124), .c(new_n125), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  tech160nm_fixnrc02aa1n02p5x5 g032(.a(\b[9] ), .b(\a[10] ), .out0(new_n128));
  norb02aa1n02x5               g033(.a(new_n125), .b(new_n128), .out0(new_n129));
  aob012aa1n02x5               g034(.a(new_n98), .b(\b[9] ), .c(\a[10] ), .out0(new_n130));
  oaib12aa1n09x5               g035(.a(new_n130), .b(\b[9] ), .c(new_n97), .out0(new_n131));
  tech160nm_fiao0012aa1n02p5x5 g036(.a(new_n131), .b(new_n124), .c(new_n129), .o(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n04x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand02aa1n06x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoi012aa1n02x5               g040(.a(new_n134), .b(new_n132), .c(new_n135), .o1(new_n136));
  xnrb03aa1n02x5               g041(.a(new_n136), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor042aa1n03x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nano23aa1d15x5               g044(.a(new_n134), .b(new_n138), .c(new_n139), .d(new_n135), .out0(new_n140));
  tech160nm_fiao0012aa1n02p5x5 g045(.a(new_n138), .b(new_n134), .c(new_n139), .o(new_n141));
  tech160nm_fiao0012aa1n02p5x5 g046(.a(new_n141), .b(new_n140), .c(new_n131), .o(new_n142));
  nanb03aa1d24x5               g047(.a(new_n128), .b(new_n140), .c(new_n125), .out0(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  xnrc02aa1n12x5               g049(.a(\b[12] ), .b(\a[13] ), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n145), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n142), .c(new_n124), .d(new_n144), .o1(new_n147));
  aoi112aa1n02x5               g052(.a(new_n142), .b(new_n146), .c(new_n124), .d(new_n144), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n147), .b(new_n148), .out0(\s[13] ));
  orn002aa1n02x5               g054(.a(\a[13] ), .b(\b[12] ), .o(new_n150));
  tech160nm_fixnrc02aa1n04x5   g055(.a(\b[13] ), .b(\a[14] ), .out0(new_n151));
  xobna2aa1n03x5               g056(.a(new_n151), .b(new_n147), .c(new_n150), .out0(\s[14] ));
  nor042aa1n03x5               g057(.a(new_n151), .b(new_n145), .o1(new_n153));
  aoai13aa1n04x5               g058(.a(new_n153), .b(new_n141), .c(new_n140), .d(new_n131), .o1(new_n154));
  oao003aa1n02x5               g059(.a(\a[14] ), .b(\b[13] ), .c(new_n150), .carry(new_n155));
  nand02aa1n02x5               g060(.a(new_n154), .b(new_n155), .o1(new_n156));
  aoi013aa1n02x4               g061(.a(new_n156), .b(new_n124), .c(new_n144), .d(new_n153), .o1(new_n157));
  xnrb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  oaoi03aa1n02x5               g063(.a(\a[15] ), .b(\b[14] ), .c(new_n157), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[15] ), .c(\a[16] ), .out0(\s[16] ));
  inv000aa1d42x5               g065(.a(\a[17] ), .o1(new_n161));
  tech160nm_fixnrc02aa1n02p5x5 g066(.a(\b[14] ), .b(\a[15] ), .out0(new_n162));
  xnrc02aa1n06x5               g067(.a(\b[15] ), .b(\a[16] ), .out0(new_n163));
  nor042aa1n04x5               g068(.a(new_n163), .b(new_n162), .o1(new_n164));
  nano22aa1n12x5               g069(.a(new_n143), .b(new_n153), .c(new_n164), .out0(new_n165));
  inv000aa1d42x5               g070(.a(new_n164), .o1(new_n166));
  tech160nm_fiaoi012aa1n04x5   g071(.a(new_n166), .b(new_n154), .c(new_n155), .o1(new_n167));
  inv000aa1d42x5               g072(.a(\a[16] ), .o1(new_n168));
  inv000aa1d42x5               g073(.a(\b[15] ), .o1(new_n169));
  norp02aa1n02x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  tech160nm_fioaoi03aa1n02p5x5 g075(.a(new_n168), .b(new_n169), .c(new_n170), .o1(new_n171));
  inv000aa1n02x5               g076(.a(new_n171), .o1(new_n172));
  aoi112aa1n09x5               g077(.a(new_n167), .b(new_n172), .c(new_n124), .d(new_n165), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[16] ), .c(new_n161), .out0(\s[17] ));
  oaoi03aa1n03x5               g079(.a(\a[17] ), .b(\b[16] ), .c(new_n173), .o1(new_n175));
  xorb03aa1n02x5               g080(.a(new_n175), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv040aa1d32x5               g081(.a(\a[18] ), .o1(new_n177));
  xroi22aa1d06x4               g082(.a(new_n161), .b(\b[16] ), .c(new_n177), .d(\b[17] ), .out0(new_n178));
  inv030aa1n02x5               g083(.a(new_n178), .o1(new_n179));
  aoi112aa1n03x5               g084(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n180));
  aoib12aa1n06x5               g085(.a(new_n180), .b(new_n177), .c(\b[17] ), .out0(new_n181));
  tech160nm_fioai012aa1n05x5   g086(.a(new_n181), .b(new_n173), .c(new_n179), .o1(new_n182));
  xorb03aa1n02x5               g087(.a(new_n182), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g088(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n09x5               g089(.a(\b[18] ), .b(\a[19] ), .o1(new_n185));
  nand22aa1n04x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n186), .b(new_n185), .out0(new_n187));
  nor002aa1n06x5               g092(.a(\b[19] ), .b(\a[20] ), .o1(new_n188));
  nand22aa1n09x5               g093(.a(\b[19] ), .b(\a[20] ), .o1(new_n189));
  nanb02aa1n02x5               g094(.a(new_n188), .b(new_n189), .out0(new_n190));
  aoai13aa1n03x5               g095(.a(new_n190), .b(new_n185), .c(new_n182), .d(new_n187), .o1(new_n191));
  nand22aa1n03x5               g096(.a(new_n124), .b(new_n165), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n156), .b(new_n164), .o1(new_n193));
  nand23aa1n06x5               g098(.a(new_n192), .b(new_n193), .c(new_n171), .o1(new_n194));
  nor042aa1n03x5               g099(.a(\b[16] ), .b(\a[17] ), .o1(new_n195));
  aob012aa1n02x5               g100(.a(new_n195), .b(\b[17] ), .c(\a[18] ), .out0(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(\b[17] ), .c(new_n177), .out0(new_n197));
  aoai13aa1n02x5               g102(.a(new_n187), .b(new_n197), .c(new_n194), .d(new_n178), .o1(new_n198));
  nona22aa1n03x5               g103(.a(new_n198), .b(new_n190), .c(new_n185), .out0(new_n199));
  nanp02aa1n03x5               g104(.a(new_n191), .b(new_n199), .o1(\s[20] ));
  nona23aa1n09x5               g105(.a(new_n189), .b(new_n186), .c(new_n185), .d(new_n188), .out0(new_n201));
  ao0012aa1n03x7               g106(.a(new_n188), .b(new_n185), .c(new_n189), .o(new_n202));
  oabi12aa1n18x5               g107(.a(new_n202), .b(new_n201), .c(new_n181), .out0(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  norb02aa1n03x5               g109(.a(new_n178), .b(new_n201), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  tech160nm_fioai012aa1n05x5   g111(.a(new_n204), .b(new_n173), .c(new_n206), .o1(new_n207));
  xorb03aa1n02x5               g112(.a(new_n207), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n02x5               g113(.a(\b[20] ), .b(\a[21] ), .o1(new_n209));
  xnrc02aa1n12x5               g114(.a(\b[20] ), .b(\a[21] ), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  xnrc02aa1n12x5               g116(.a(\b[21] ), .b(\a[22] ), .out0(new_n212));
  aoai13aa1n03x5               g117(.a(new_n212), .b(new_n209), .c(new_n207), .d(new_n211), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n211), .b(new_n203), .c(new_n194), .d(new_n205), .o1(new_n214));
  nona22aa1n03x5               g119(.a(new_n214), .b(new_n212), .c(new_n209), .out0(new_n215));
  nanp02aa1n03x5               g120(.a(new_n213), .b(new_n215), .o1(\s[22] ));
  nor042aa1n06x5               g121(.a(new_n212), .b(new_n210), .o1(new_n217));
  inv000aa1d42x5               g122(.a(\a[22] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\b[21] ), .o1(new_n219));
  oaoi03aa1n12x5               g124(.a(new_n218), .b(new_n219), .c(new_n209), .o1(new_n220));
  inv000aa1d42x5               g125(.a(new_n220), .o1(new_n221));
  tech160nm_fiaoi012aa1n03p5x5 g126(.a(new_n221), .b(new_n203), .c(new_n217), .o1(new_n222));
  nano23aa1n06x5               g127(.a(new_n185), .b(new_n188), .c(new_n189), .d(new_n186), .out0(new_n223));
  nano22aa1n12x5               g128(.a(new_n179), .b(new_n217), .c(new_n223), .out0(new_n224));
  inv000aa1d42x5               g129(.a(new_n224), .o1(new_n225));
  tech160nm_fioai012aa1n05x5   g130(.a(new_n222), .b(new_n173), .c(new_n225), .o1(new_n226));
  xorb03aa1n02x5               g131(.a(new_n226), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g132(.a(\b[22] ), .b(\a[23] ), .o1(new_n228));
  xorc02aa1n12x5               g133(.a(\a[23] ), .b(\b[22] ), .out0(new_n229));
  xnrc02aa1n12x5               g134(.a(\b[23] ), .b(\a[24] ), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n228), .c(new_n226), .d(new_n229), .o1(new_n231));
  inv000aa1n02x5               g136(.a(new_n222), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n229), .b(new_n232), .c(new_n194), .d(new_n224), .o1(new_n233));
  nona22aa1n03x5               g138(.a(new_n233), .b(new_n230), .c(new_n228), .out0(new_n234));
  nanp02aa1n03x5               g139(.a(new_n231), .b(new_n234), .o1(\s[24] ));
  norb02aa1n03x5               g140(.a(new_n229), .b(new_n230), .out0(new_n236));
  inv040aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  nano32aa1n02x4               g142(.a(new_n237), .b(new_n178), .c(new_n217), .d(new_n223), .out0(new_n238));
  inv000aa1n02x5               g143(.a(new_n238), .o1(new_n239));
  aoai13aa1n06x5               g144(.a(new_n217), .b(new_n202), .c(new_n223), .d(new_n197), .o1(new_n240));
  orn002aa1n02x5               g145(.a(\a[23] ), .b(\b[22] ), .o(new_n241));
  oao003aa1n02x5               g146(.a(\a[24] ), .b(\b[23] ), .c(new_n241), .carry(new_n242));
  aoai13aa1n12x5               g147(.a(new_n242), .b(new_n237), .c(new_n240), .d(new_n220), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n243), .o1(new_n244));
  tech160nm_fioai012aa1n05x5   g149(.a(new_n244), .b(new_n173), .c(new_n239), .o1(new_n245));
  xorb03aa1n02x5               g150(.a(new_n245), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g151(.a(\b[24] ), .b(\a[25] ), .o1(new_n247));
  xorc02aa1n06x5               g152(.a(\a[25] ), .b(\b[24] ), .out0(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[25] ), .b(\a[26] ), .out0(new_n249));
  aoai13aa1n03x5               g154(.a(new_n249), .b(new_n247), .c(new_n245), .d(new_n248), .o1(new_n250));
  aoai13aa1n02x5               g155(.a(new_n248), .b(new_n243), .c(new_n194), .d(new_n238), .o1(new_n251));
  nona22aa1n02x4               g156(.a(new_n251), .b(new_n249), .c(new_n247), .out0(new_n252));
  nanp02aa1n03x5               g157(.a(new_n250), .b(new_n252), .o1(\s[26] ));
  norb02aa1n02x5               g158(.a(new_n248), .b(new_n249), .out0(new_n254));
  inv000aa1d42x5               g159(.a(\a[26] ), .o1(new_n255));
  inv000aa1d42x5               g160(.a(\b[25] ), .o1(new_n256));
  oaoi03aa1n12x5               g161(.a(new_n255), .b(new_n256), .c(new_n247), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  aoi012aa1n06x5               g163(.a(new_n258), .b(new_n243), .c(new_n254), .o1(new_n259));
  inv000aa1n02x5               g164(.a(new_n254), .o1(new_n260));
  nona22aa1d18x5               g165(.a(new_n224), .b(new_n260), .c(new_n237), .out0(new_n261));
  oai012aa1n06x5               g166(.a(new_n259), .b(new_n173), .c(new_n261), .o1(new_n262));
  xorb03aa1n03x5               g167(.a(new_n262), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g168(.a(\b[26] ), .b(\a[27] ), .o1(new_n264));
  xorc02aa1n02x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[27] ), .b(\a[28] ), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n264), .c(new_n262), .d(new_n265), .o1(new_n267));
  aoai13aa1n06x5               g172(.a(new_n236), .b(new_n221), .c(new_n203), .d(new_n217), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n257), .b(new_n260), .c(new_n268), .d(new_n242), .o1(new_n269));
  inv040aa1d30x5               g174(.a(new_n261), .o1(new_n270));
  aoai13aa1n02x5               g175(.a(new_n265), .b(new_n269), .c(new_n194), .d(new_n270), .o1(new_n271));
  nona22aa1n02x4               g176(.a(new_n271), .b(new_n266), .c(new_n264), .out0(new_n272));
  nanp02aa1n03x5               g177(.a(new_n267), .b(new_n272), .o1(\s[28] ));
  norb02aa1n02x5               g178(.a(new_n265), .b(new_n266), .out0(new_n274));
  aoai13aa1n03x5               g179(.a(new_n274), .b(new_n269), .c(new_n194), .d(new_n270), .o1(new_n275));
  aob012aa1n03x5               g180(.a(new_n264), .b(\b[27] ), .c(\a[28] ), .out0(new_n276));
  oa0012aa1n12x5               g181(.a(new_n276), .b(\b[27] ), .c(\a[28] ), .o(new_n277));
  inv000aa1d42x5               g182(.a(new_n277), .o1(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[28] ), .b(\a[29] ), .out0(new_n279));
  nona22aa1n03x5               g184(.a(new_n275), .b(new_n278), .c(new_n279), .out0(new_n280));
  aoai13aa1n03x5               g185(.a(new_n279), .b(new_n278), .c(new_n262), .d(new_n274), .o1(new_n281));
  nanp02aa1n03x5               g186(.a(new_n281), .b(new_n280), .o1(\s[29] ));
  xorb03aa1n02x5               g187(.a(new_n115), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g188(.a(new_n265), .b(new_n279), .c(new_n266), .out0(new_n284));
  oaoi03aa1n09x5               g189(.a(\a[29] ), .b(\b[28] ), .c(new_n277), .o1(new_n285));
  xnrc02aa1n02x5               g190(.a(\b[29] ), .b(\a[30] ), .out0(new_n286));
  aoai13aa1n03x5               g191(.a(new_n286), .b(new_n285), .c(new_n262), .d(new_n284), .o1(new_n287));
  aoai13aa1n02x7               g192(.a(new_n284), .b(new_n269), .c(new_n194), .d(new_n270), .o1(new_n288));
  nona22aa1n02x4               g193(.a(new_n288), .b(new_n285), .c(new_n286), .out0(new_n289));
  nanp02aa1n03x5               g194(.a(new_n287), .b(new_n289), .o1(\s[30] ));
  nanb02aa1n02x5               g195(.a(new_n286), .b(new_n285), .out0(new_n291));
  oai012aa1n02x5               g196(.a(new_n291), .b(\b[29] ), .c(\a[30] ), .o1(new_n292));
  norb02aa1n02x5               g197(.a(new_n284), .b(new_n286), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n269), .c(new_n194), .d(new_n270), .o1(new_n294));
  xnrc02aa1n02x5               g199(.a(\b[30] ), .b(\a[31] ), .out0(new_n295));
  nona22aa1n02x4               g200(.a(new_n294), .b(new_n295), .c(new_n292), .out0(new_n296));
  aoai13aa1n03x5               g201(.a(new_n295), .b(new_n292), .c(new_n262), .d(new_n293), .o1(new_n297));
  nanp02aa1n03x5               g202(.a(new_n297), .b(new_n296), .o1(\s[31] ));
  xorb03aa1n02x5               g203(.a(new_n117), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g204(.a(new_n111), .b(new_n117), .c(new_n112), .o1(new_n300));
  xnrb03aa1n02x5               g205(.a(new_n300), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  nanp02aa1n02x5               g206(.a(new_n118), .b(new_n120), .o1(new_n302));
  xorb03aa1n02x5               g207(.a(new_n302), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n02x5               g208(.a(new_n106), .b(new_n302), .c(new_n121), .o1(new_n304));
  xnrb03aa1n02x5               g209(.a(new_n304), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g210(.a(new_n107), .b(new_n302), .c(new_n122), .o(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g212(.a(new_n101), .b(new_n306), .c(new_n102), .o1(new_n308));
  xnrb03aa1n02x5               g213(.a(new_n308), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g214(.a(new_n124), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


