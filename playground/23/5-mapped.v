// Benchmark "adder" written by ABC on Wed Jul 17 23:45:24 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n158, new_n159, new_n160, new_n161, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n176, new_n177, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n189, new_n190, new_n191, new_n192, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n202, new_n203, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n211, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n220, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n237,
    new_n238, new_n239, new_n240, new_n241, new_n242, new_n243, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n263, new_n264, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n275, new_n276,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n311, new_n314, new_n316, new_n317, new_n318, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  tech160nm_fixorc02aa1n03p5x5 g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[8] ), .o1(new_n99));
  nor002aa1n02x5               g004(.a(\b[3] ), .b(\a[4] ), .o1(new_n100));
  nand42aa1n02x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nor022aa1n04x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nano23aa1n02x4               g008(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n104));
  and002aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o(new_n105));
  nanp02aa1n02x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  nor002aa1n02x5               g011(.a(\b[1] ), .b(\a[2] ), .o1(new_n107));
  oab012aa1n02x4               g012(.a(new_n105), .b(new_n107), .c(new_n106), .out0(new_n108));
  nanp02aa1n02x5               g013(.a(new_n104), .b(new_n108), .o1(new_n109));
  aoi012aa1n02x7               g014(.a(new_n100), .b(new_n102), .c(new_n101), .o1(new_n110));
  nor042aa1n02x5               g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor042aa1n12x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand22aa1n03x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nano23aa1n06x5               g019(.a(new_n111), .b(new_n113), .c(new_n114), .d(new_n112), .out0(new_n115));
  xorc02aa1n02x5               g020(.a(\a[6] ), .b(\b[5] ), .out0(new_n116));
  xorc02aa1n02x5               g021(.a(\a[5] ), .b(\b[4] ), .out0(new_n117));
  nanp03aa1n02x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\a[6] ), .o1(new_n119));
  inv000aa1d42x5               g024(.a(\b[5] ), .o1(new_n120));
  nor002aa1n02x5               g025(.a(\b[4] ), .b(\a[5] ), .o1(new_n121));
  oao003aa1n02x5               g026(.a(new_n119), .b(new_n120), .c(new_n121), .carry(new_n122));
  inv040aa1n03x5               g027(.a(new_n113), .o1(new_n123));
  oaoi03aa1n02x5               g028(.a(\a[8] ), .b(\b[7] ), .c(new_n123), .o1(new_n124));
  tech160nm_fiaoi012aa1n03p5x5 g029(.a(new_n124), .b(new_n115), .c(new_n122), .o1(new_n125));
  aoai13aa1n06x5               g030(.a(new_n125), .b(new_n118), .c(new_n109), .d(new_n110), .o1(new_n126));
  tech160nm_fioaoi03aa1n03p5x5 g031(.a(new_n98), .b(new_n99), .c(new_n126), .o1(new_n127));
  xnrc02aa1n02x5               g032(.a(new_n127), .b(new_n97), .out0(\s[10] ));
  tech160nm_fioaoi03aa1n03p5x5 g033(.a(\a[10] ), .b(\b[9] ), .c(new_n127), .o1(new_n129));
  xorb03aa1n02x5               g034(.a(new_n129), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n03x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  nand42aa1n03x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  norb02aa1n02x5               g037(.a(new_n132), .b(new_n131), .out0(new_n133));
  nor042aa1n02x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nand42aa1n03x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  aoai13aa1n03x5               g041(.a(new_n136), .b(new_n131), .c(new_n129), .d(new_n133), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(new_n129), .b(new_n133), .o1(new_n138));
  nona22aa1n02x4               g043(.a(new_n138), .b(new_n136), .c(new_n131), .out0(new_n139));
  nanp02aa1n02x5               g044(.a(new_n139), .b(new_n137), .o1(\s[12] ));
  nano23aa1n09x5               g045(.a(new_n131), .b(new_n134), .c(new_n135), .d(new_n132), .out0(new_n141));
  nanp02aa1n02x5               g046(.a(new_n99), .b(new_n98), .o1(new_n142));
  oaoi03aa1n02x5               g047(.a(\a[10] ), .b(\b[9] ), .c(new_n142), .o1(new_n143));
  aoi012aa1n03x5               g048(.a(new_n134), .b(new_n131), .c(new_n135), .o1(new_n144));
  aobi12aa1n09x5               g049(.a(new_n144), .b(new_n141), .c(new_n143), .out0(new_n145));
  nona23aa1n03x5               g050(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n146));
  oabi12aa1n03x5               g051(.a(new_n105), .b(new_n106), .c(new_n107), .out0(new_n147));
  tech160nm_fioai012aa1n04x5   g052(.a(new_n110), .b(new_n146), .c(new_n147), .o1(new_n148));
  nona23aa1n09x5               g053(.a(new_n114), .b(new_n112), .c(new_n111), .d(new_n113), .out0(new_n149));
  nano22aa1n03x7               g054(.a(new_n149), .b(new_n116), .c(new_n117), .out0(new_n150));
  oaoi03aa1n02x5               g055(.a(new_n119), .b(new_n120), .c(new_n121), .o1(new_n151));
  oabi12aa1n06x5               g056(.a(new_n124), .b(new_n149), .c(new_n151), .out0(new_n152));
  xorc02aa1n02x5               g057(.a(\a[9] ), .b(\b[8] ), .out0(new_n153));
  and003aa1n02x5               g058(.a(new_n141), .b(new_n97), .c(new_n153), .o(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n152), .c(new_n150), .d(new_n148), .o1(new_n155));
  xorc02aa1n06x5               g060(.a(\a[13] ), .b(\b[12] ), .out0(new_n156));
  xnbna2aa1n03x5               g061(.a(new_n156), .b(new_n155), .c(new_n145), .out0(\s[13] ));
  inv000aa1d42x5               g062(.a(\a[14] ), .o1(new_n158));
  nanp02aa1n02x5               g063(.a(new_n155), .b(new_n145), .o1(new_n159));
  nor042aa1n03x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  aoi012aa1n02x5               g065(.a(new_n160), .b(new_n159), .c(new_n156), .o1(new_n161));
  xorb03aa1n02x5               g066(.a(new_n161), .b(\b[13] ), .c(new_n158), .out0(\s[14] ));
  xorc02aa1n03x5               g067(.a(\a[14] ), .b(\b[13] ), .out0(new_n163));
  and002aa1n02x5               g068(.a(new_n163), .b(new_n156), .o(new_n164));
  inv000aa1d42x5               g069(.a(new_n164), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\b[13] ), .o1(new_n166));
  oao003aa1n12x5               g071(.a(new_n158), .b(new_n166), .c(new_n160), .carry(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n04x5               g073(.a(new_n168), .b(new_n165), .c(new_n155), .d(new_n145), .o1(new_n169));
  xorb03aa1n02x5               g074(.a(new_n169), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n03x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  nand42aa1n04x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nor042aa1n02x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  nand42aa1n03x5               g078(.a(\b[15] ), .b(\a[16] ), .o1(new_n174));
  nanb02aa1n02x5               g079(.a(new_n173), .b(new_n174), .out0(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n176));
  aoi112aa1n03x5               g081(.a(new_n175), .b(new_n171), .c(new_n169), .d(new_n172), .o1(new_n177));
  nanb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(\s[16] ));
  nano23aa1n09x5               g083(.a(new_n171), .b(new_n173), .c(new_n174), .d(new_n172), .out0(new_n179));
  nand23aa1n06x5               g084(.a(new_n179), .b(new_n156), .c(new_n163), .o1(new_n180));
  nano32aa1n03x7               g085(.a(new_n180), .b(new_n153), .c(new_n141), .d(new_n97), .out0(new_n181));
  aoai13aa1n12x5               g086(.a(new_n181), .b(new_n152), .c(new_n148), .d(new_n150), .o1(new_n182));
  tech160nm_fiao0012aa1n02p5x5 g087(.a(new_n173), .b(new_n171), .c(new_n174), .o(new_n183));
  aoi012aa1n06x5               g088(.a(new_n183), .b(new_n179), .c(new_n167), .o1(new_n184));
  inv020aa1n03x5               g089(.a(new_n184), .o1(new_n185));
  oab012aa1d15x5               g090(.a(new_n185), .b(new_n145), .c(new_n180), .out0(new_n186));
  nanp02aa1n06x5               g091(.a(new_n182), .b(new_n186), .o1(new_n187));
  xorb03aa1n02x5               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g093(.a(\a[18] ), .o1(new_n189));
  inv040aa1d30x5               g094(.a(\a[17] ), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\b[16] ), .o1(new_n191));
  oaoi03aa1n03x5               g096(.a(new_n190), .b(new_n191), .c(new_n187), .o1(new_n192));
  xorb03aa1n02x5               g097(.a(new_n192), .b(\b[17] ), .c(new_n189), .out0(\s[18] ));
  xroi22aa1d06x4               g098(.a(new_n190), .b(\b[16] ), .c(new_n189), .d(\b[17] ), .out0(new_n194));
  inv000aa1d42x5               g099(.a(new_n194), .o1(new_n195));
  nor042aa1n02x5               g100(.a(\b[17] ), .b(\a[18] ), .o1(new_n196));
  aoi112aa1n09x5               g101(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n197));
  norp02aa1n06x5               g102(.a(new_n197), .b(new_n196), .o1(new_n198));
  aoai13aa1n04x5               g103(.a(new_n198), .b(new_n195), .c(new_n182), .d(new_n186), .o1(new_n199));
  xorb03aa1n02x5               g104(.a(new_n199), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g105(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g106(.a(\b[18] ), .b(\a[19] ), .o1(new_n202));
  nand02aa1n12x5               g107(.a(\b[18] ), .b(\a[19] ), .o1(new_n203));
  nor042aa1d18x5               g108(.a(\b[19] ), .b(\a[20] ), .o1(new_n204));
  nand02aa1d28x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  norb02aa1n15x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  inv000aa1d42x5               g111(.a(new_n206), .o1(new_n207));
  aoai13aa1n03x5               g112(.a(new_n207), .b(new_n202), .c(new_n199), .d(new_n203), .o1(new_n208));
  norb02aa1n06x4               g113(.a(new_n203), .b(new_n202), .out0(new_n209));
  nand22aa1n02x5               g114(.a(new_n199), .b(new_n209), .o1(new_n210));
  nona22aa1n02x4               g115(.a(new_n210), .b(new_n207), .c(new_n202), .out0(new_n211));
  nanp02aa1n03x5               g116(.a(new_n211), .b(new_n208), .o1(\s[20] ));
  nona23aa1d18x5               g117(.a(new_n205), .b(new_n203), .c(new_n202), .d(new_n204), .out0(new_n213));
  tech160nm_fiaoi012aa1n05x5   g118(.a(new_n204), .b(new_n202), .c(new_n205), .o1(new_n214));
  oai012aa1n12x5               g119(.a(new_n214), .b(new_n213), .c(new_n198), .o1(new_n215));
  inv000aa1d42x5               g120(.a(new_n215), .o1(new_n216));
  nanb02aa1n06x5               g121(.a(new_n213), .b(new_n194), .out0(new_n217));
  aoai13aa1n04x5               g122(.a(new_n216), .b(new_n217), .c(new_n182), .d(new_n186), .o1(new_n218));
  xorb03aa1n02x5               g123(.a(new_n218), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor022aa1n16x5               g124(.a(\b[20] ), .b(\a[21] ), .o1(new_n220));
  xorc02aa1n12x5               g125(.a(\a[21] ), .b(\b[20] ), .out0(new_n221));
  xnrc02aa1n12x5               g126(.a(\b[21] ), .b(\a[22] ), .out0(new_n222));
  aoai13aa1n03x5               g127(.a(new_n222), .b(new_n220), .c(new_n218), .d(new_n221), .o1(new_n223));
  nand22aa1n02x5               g128(.a(new_n218), .b(new_n221), .o1(new_n224));
  nona22aa1n02x4               g129(.a(new_n224), .b(new_n222), .c(new_n220), .out0(new_n225));
  nanp02aa1n03x5               g130(.a(new_n225), .b(new_n223), .o1(\s[22] ));
  oai112aa1n06x5               g131(.a(new_n209), .b(new_n206), .c(new_n197), .d(new_n196), .o1(new_n227));
  nanb02aa1n06x5               g132(.a(new_n222), .b(new_n221), .out0(new_n228));
  inv000aa1d42x5               g133(.a(\a[22] ), .o1(new_n229));
  inv040aa1d28x5               g134(.a(\b[21] ), .o1(new_n230));
  oaoi03aa1n12x5               g135(.a(new_n229), .b(new_n230), .c(new_n220), .o1(new_n231));
  aoai13aa1n12x5               g136(.a(new_n231), .b(new_n228), .c(new_n227), .d(new_n214), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  nona23aa1n08x5               g138(.a(new_n194), .b(new_n221), .c(new_n222), .d(new_n213), .out0(new_n234));
  aoai13aa1n04x5               g139(.a(new_n233), .b(new_n234), .c(new_n182), .d(new_n186), .o1(new_n235));
  xorb03aa1n02x5               g140(.a(new_n235), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g141(.a(\b[22] ), .b(\a[23] ), .o1(new_n237));
  xorc02aa1n02x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  xorc02aa1n12x5               g143(.a(\a[24] ), .b(\b[23] ), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  aoai13aa1n03x5               g145(.a(new_n240), .b(new_n237), .c(new_n235), .d(new_n238), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n235), .b(new_n238), .o1(new_n242));
  nona22aa1n02x5               g147(.a(new_n242), .b(new_n240), .c(new_n237), .out0(new_n243));
  nanp02aa1n03x5               g148(.a(new_n243), .b(new_n241), .o1(\s[24] ));
  oai012aa1n02x7               g149(.a(new_n184), .b(new_n145), .c(new_n180), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n221), .b(new_n222), .out0(new_n246));
  and002aa1n02x5               g151(.a(new_n239), .b(new_n238), .o(new_n247));
  nano22aa1n03x7               g152(.a(new_n217), .b(new_n247), .c(new_n246), .out0(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n245), .c(new_n126), .d(new_n181), .o1(new_n249));
  inv000aa1d42x5               g154(.a(new_n231), .o1(new_n250));
  aoai13aa1n06x5               g155(.a(new_n247), .b(new_n250), .c(new_n215), .d(new_n246), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\a[24] ), .o1(new_n252));
  inv000aa1d42x5               g157(.a(\b[23] ), .o1(new_n253));
  oaoi03aa1n12x5               g158(.a(new_n252), .b(new_n253), .c(new_n237), .o1(new_n254));
  nanp02aa1n03x5               g159(.a(new_n251), .b(new_n254), .o1(new_n255));
  nanb02aa1n03x5               g160(.a(new_n255), .b(new_n249), .out0(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g162(.a(\b[24] ), .b(\a[25] ), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  xorc02aa1n12x5               g164(.a(\a[26] ), .b(\b[25] ), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  aoai13aa1n03x5               g166(.a(new_n261), .b(new_n258), .c(new_n256), .d(new_n259), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n259), .b(new_n255), .c(new_n187), .d(new_n248), .o1(new_n263));
  nona22aa1n02x5               g168(.a(new_n263), .b(new_n261), .c(new_n258), .out0(new_n264));
  nanp02aa1n03x5               g169(.a(new_n262), .b(new_n264), .o1(\s[26] ));
  and002aa1n06x5               g170(.a(new_n260), .b(new_n259), .o(new_n266));
  nano22aa1n03x7               g171(.a(new_n234), .b(new_n247), .c(new_n266), .out0(new_n267));
  aoai13aa1n06x5               g172(.a(new_n267), .b(new_n245), .c(new_n126), .d(new_n181), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n254), .o1(new_n269));
  aoai13aa1n04x5               g174(.a(new_n266), .b(new_n269), .c(new_n232), .d(new_n247), .o1(new_n270));
  aoi112aa1n02x5               g175(.a(\b[24] ), .b(\a[25] ), .c(\a[26] ), .d(\b[25] ), .o1(new_n271));
  oab012aa1n02x4               g176(.a(new_n271), .b(\a[26] ), .c(\b[25] ), .out0(new_n272));
  nand43aa1n03x5               g177(.a(new_n268), .b(new_n270), .c(new_n272), .o1(new_n273));
  xorb03aa1n03x5               g178(.a(new_n273), .b(\b[26] ), .c(\a[27] ), .out0(\s[27] ));
  norp02aa1n02x5               g179(.a(\b[26] ), .b(\a[27] ), .o1(new_n275));
  xorc02aa1n02x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnrc02aa1n02x5               g181(.a(\b[27] ), .b(\a[28] ), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n275), .c(new_n273), .d(new_n276), .o1(new_n278));
  aobi12aa1n06x5               g183(.a(new_n267), .b(new_n182), .c(new_n186), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n266), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n272), .b(new_n280), .c(new_n251), .d(new_n254), .o1(new_n281));
  oaih12aa1n02x5               g186(.a(new_n276), .b(new_n281), .c(new_n279), .o1(new_n282));
  nona22aa1n02x5               g187(.a(new_n282), .b(new_n277), .c(new_n275), .out0(new_n283));
  nanp02aa1n03x5               g188(.a(new_n278), .b(new_n283), .o1(\s[28] ));
  norb02aa1n02x5               g189(.a(new_n276), .b(new_n277), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n281), .c(new_n279), .o1(new_n286));
  aob012aa1n03x5               g191(.a(new_n275), .b(\b[27] ), .c(\a[28] ), .out0(new_n287));
  oa0012aa1n12x5               g192(.a(new_n287), .b(\b[27] ), .c(\a[28] ), .o(new_n288));
  inv000aa1d42x5               g193(.a(new_n288), .o1(new_n289));
  xnrc02aa1n02x5               g194(.a(\b[28] ), .b(\a[29] ), .out0(new_n290));
  nona22aa1n02x5               g195(.a(new_n286), .b(new_n289), .c(new_n290), .out0(new_n291));
  aoai13aa1n03x5               g196(.a(new_n290), .b(new_n289), .c(new_n273), .d(new_n285), .o1(new_n292));
  nanp02aa1n03x5               g197(.a(new_n292), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g198(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g199(.a(new_n276), .b(new_n290), .c(new_n277), .out0(new_n295));
  oaoi03aa1n09x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n288), .o1(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  aoai13aa1n03x5               g202(.a(new_n297), .b(new_n296), .c(new_n273), .d(new_n295), .o1(new_n298));
  oaih12aa1n02x5               g203(.a(new_n295), .b(new_n281), .c(new_n279), .o1(new_n299));
  nona22aa1n02x5               g204(.a(new_n299), .b(new_n296), .c(new_n297), .out0(new_n300));
  nanp02aa1n03x5               g205(.a(new_n298), .b(new_n300), .o1(\s[30] ));
  norb02aa1n02x5               g206(.a(new_n295), .b(new_n297), .out0(new_n302));
  oaih12aa1n02x5               g207(.a(new_n302), .b(new_n281), .c(new_n279), .o1(new_n303));
  nanb02aa1n02x5               g208(.a(new_n297), .b(new_n296), .out0(new_n304));
  oai012aa1n02x5               g209(.a(new_n304), .b(\b[29] ), .c(\a[30] ), .o1(new_n305));
  xnrc02aa1n02x5               g210(.a(\b[30] ), .b(\a[31] ), .out0(new_n306));
  nona22aa1n02x5               g211(.a(new_n303), .b(new_n305), .c(new_n306), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n306), .b(new_n305), .c(new_n273), .d(new_n302), .o1(new_n308));
  nanp02aa1n03x5               g213(.a(new_n308), .b(new_n307), .o1(\s[31] ));
  xorb03aa1n02x5               g214(.a(new_n108), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  aoi012aa1n02x5               g215(.a(new_n102), .b(new_n108), .c(new_n103), .o1(new_n311));
  xnrb03aa1n02x5               g216(.a(new_n311), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g217(.a(new_n148), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  aoi012aa1n03x5               g218(.a(new_n121), .b(new_n148), .c(new_n117), .o1(new_n314));
  xorb03aa1n02x5               g219(.a(new_n314), .b(\b[5] ), .c(new_n119), .out0(\s[6] ));
  and002aa1n02x5               g220(.a(\b[5] ), .b(\a[6] ), .o(new_n316));
  nanp02aa1n03x5               g221(.a(new_n314), .b(new_n116), .o1(new_n317));
  nona23aa1n06x5               g222(.a(new_n317), .b(new_n114), .c(new_n113), .d(new_n316), .out0(new_n318));
  aboi22aa1n03x5               g223(.a(new_n316), .b(new_n317), .c(new_n123), .d(new_n114), .out0(new_n319));
  norb02aa1n02x5               g224(.a(new_n318), .b(new_n319), .out0(\s[7] ));
  norb02aa1n02x5               g225(.a(new_n112), .b(new_n111), .out0(new_n321));
  xnbna2aa1n03x5               g226(.a(new_n321), .b(new_n318), .c(new_n123), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


